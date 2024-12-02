/* eslint-disable max-lines-per-function, func-style, complexity, no-var, max-lines, block-scoped-var, max-len, no-return-assign */

import bsearch from 'binary-search-bounds';
import createGeometry from './geometry';
import Graph from './graph';

let LEAF_CUTOFF = 64;
let BUCKET_SIZE = 32;

function Leaf (verts) {
	this.verts = verts;
	this.leaf = true;
}

function Bucket (y0, y1, top, bottom, left, right, on) {
	this.y0 = y0;
	this.y1 = y1;
	this.top = top;
	this.bottom = bottom;
	this.left = left;
	this.right = right;
	this.on = on;
}

function Node (x, buckets, left, right) {
	this.x = x;
	this.buckets = buckets;
	this.left = left;
	this.right = right;
}

function L1PathPlanner (geometry, graph, root) {
	this.geometry = geometry;
	this.graph = graph;
	this.root = root;
}

let proto = L1PathPlanner.prototype;

function compareBucket (bucket, y) {
	return bucket.y0 - y;
}

function connectList (nodes, geom, graph, target, x, y) {
	for (let i = 0; i < nodes.length; ++i) {
		let v = nodes[i];
		if (!geom.stabBox(v.x, v.y, x, y)) {
			if (target)
				graph.addT(v);
			else
				graph.addS(v);
		}
	}
}

function connectNodes (geom, graph, node, target, x, y) {
	//Mark target nodes
	while (node) {
		//Check leaf case
		if (node.leaf) {
			let vv = node.verts;
			let nn = vv.length;
			for (let i = 0; i < nn; ++i) {
				var v = vv[i];
				if (!geom.stabBox(v.x, v.y, x, y)) {
					if (target)
						graph.addT(v);
					else
						graph.addS(v);
				}
			}
			break;
		}

		//Otherwise, glue into buckets
		let buckets = node.buckets;
		let idx = bsearch.lt(buckets, y, compareBucket);
		if (idx >= 0) {
			let bb = buckets[idx];
			if (y < bb.y1) {
				//Common case:
				if (node.x >= x) {
					//Connect right
					connectList(bb.right, geom, graph, target, x, y);
				}
				if (node.x <= x) {
					//Connect left
					connectList(bb.left, geom, graph, target, x, y);
				}
				//Connect on
				connectList(bb.on, geom, graph, target, x, y);
			} else {
				//Connect to bottom of bucket above
				var v = buckets[idx].bottom;
				if (v && !geom.stabBox(v.x, v.y, x, y)) {
					if (target)
						graph.addT(v);
					else
						graph.addS(v);
				}
				//Connect to top of bucket below
				if (idx + 1 < buckets.length) {
					var v = buckets[idx + 1].top;
					if (v && !geom.stabBox(v.x, v.y, x, y)) {
						if (target)
							graph.addT(v);
						else
							graph.addS(v);
					}
				}
			}
		} else {
			//Connect to top of box
			var v = buckets[0].top;
			if (v && !geom.stabBox(v.x, v.y, x, y)) {
				if (target)
					graph.addT(v);
				else
					graph.addS(v);
			}
		}
		if (node.x > x)
			node = node.left;
		else if (node.x < x)
			node = node.right;
		else
			break;
	}
}

proto.search = function (tx, ty, sx, sy, out) {
	let geom = this.geometry;

	//Degenerate case:  s and t are equal
	if (tx === sx && ty === sy) {
		if (!geom.stabBox(tx, ty, sx, sy)) {
			if (out)
				out.push(sx, sy);

			return 0;
		}

		return Infinity;
	}

	//Check easy case - s and t directly connected
	if (!geom.stabBox(tx, ty, sx, sy)) {
		if (out) {
			if (sx !== tx && sy !== ty)
				out.push(tx, ty, sx, ty, sx, sy);
			else
				out.push(tx, ty, sx, sy);
		}

		return Math.abs(tx - sx) + Math.abs(ty - sy);
	}

	//Prepare graph
	let graph = this.graph;
	graph.setSourceAndTarget(sx, sy, tx, ty);

	//Mark target
	connectNodes(geom, graph, this.root, true, tx, ty);

	//Mark source
	connectNodes(geom, graph, this.root, false, sx, sy);

	//Run A*
	let dist = graph.search();

	//Recover path
	if (out && dist < Infinity)
		graph.getPath(out);

	return dist;
};

function comparePair (a, b) {
	let d = a[1] - b[1];
	if (d)
		return d;

	return a[0] - b[0];
}

function makePartition (x, corners, geom, edges) {
	let left = [];
	let right = [];
	let on = [];

	//Intersect rays along x horizontal line
	for (var i = 0; i < corners.length; ++i) {
		let c = corners[i];
		if (!geom.stabRay(c[0], c[1], x))
			on.push(c);

		if (c[0] < x)
			left.push(c);
		else if (c[0] > x)
			right.push(c);
	}

	//Sort on events by y then x
	on.sort(comparePair);

	//Construct vertices and horizontal edges
	let vis = [];
	let rem = [];
	for (var i = 0; i < on.length;) {
		let l = x;
		let r = x;
		let v = on[i];
		let y = v[1];
		while (i < on.length && on[i][1] === y && on[i][0] < x)
			l = on[i++][0];

		if (l < x)
			vis.push([l, y]);

		while (i < on.length && on[i][1] === y && on[i][0] === x) {
			rem.push(on[i]);
			vis.push(on[i]);
			++i;
		}
		if (i < on.length && on[i][1] === y) {
			r = on[i++][0];
			while (i < on.length && on[i][1] === y)
				++i;
		}
		if (r > x)
			vis.push([r, y]);
	}

	return {
		x,
		left,
		right,
		on: rem,
		vis
	};
}
window.inner = 0;
function createPlanner (grid, loops) {
	let geom = createGeometry(grid, loops);
	let graph = new Graph();
	let verts = {};
	let edges = [];

	function makeVertex (pair) {
		if (!pair)
			return null;

		let res = verts[pair];
		if (res)
			return res;

		return verts[pair] = graph.vertex(pair[0], pair[1]);
	}

	function makeLeaf (corners, x0, x1) {
		let localVerts = [];
		for (let i = 0; i < corners.length; ++i) {
			let u = corners[i];
			let ux = graph.vertex(u[0], u[1]);
			localVerts.push(ux);
			verts[u] = ux;
			for (let j = 0; j < i; ++j) {
				let v = corners[j];
				if (!geom.stabBox(u[0], u[1], v[0], v[1]))
					edges.push([u, v]);
			}
		}

		return new Leaf(localVerts);
	}

	function makeBucket (corners, x) {
		//Split visible corners into 3 cases
		let left = [];
		let right = [];
		let on = [];
		for (var i = 0; i < corners.length; ++i) {
			if (corners[i][0] < x)
				left.push(corners[i]);
			else if (corners[i][0] > x)
				right.push(corners[i]);
			else
				on.push(corners[i]);
		}

		//Add Steiner vertices if needed
		function addSteiner (y, first) {
			if (!geom.stabTile(x, y)) {
				for (let i = 0; i < on.length; ++i) {
					if (on[i][0] === x && on[i][1] === y)
						return on[i];
				}
				let pair = [x, y];
				if (first)
					on.unshift(pair);
				else
					on.push(pair);

				if (!verts[pair])
					verts[pair] = graph.vertex(x, y);

				return pair;
			}

			return null;
		}

		let y0 = corners[0][1];
		let y1 = corners[corners.length - 1][1];
		let loSteiner = addSteiner(y0, true);
		let hiSteiner = addSteiner(y1, false);

		function bipartite (a, b) {
			for (let i = 0; i < a.length; ++i) {
				let u = a[i];
				for (let j = 0; j < b.length; ++j) {
					let v = b[j];
					if (!geom.stabBox(u[0], u[1], v[0], v[1]))
						edges.push([u, v]);
				}
			}
		}

		bipartite(left, right);
		bipartite(on, left);
		bipartite(on, right);

		//Connect vertical edges
		for (var i = 1; i < on.length; ++i) {
			let u = on[i - 1];
			let v = on[i];
			if (!geom.stabBox(u[0], u[1], v[0], v[1]))
				edges.push([u, v]);
		}

		return {
			left,
			right,
			on,
			steiner0: loSteiner,
			steiner1: hiSteiner,
			y0,
			y1
		};
	}

	//Make tree
	function makeTree (corners, x0, x1) {
		if (corners.length === 0)
			return null;

		if (corners.length < LEAF_CUTOFF)
			return makeLeaf(corners, x0, x1);

		let x = corners[corners.length >>> 1][0];
		let partition = makePartition(x, corners, geom, edges);
		let left = makeTree(partition.left, x0, x);
		let right = makeTree(partition.right, x, x1);

		//Construct vertices
		for (var i = 0; i < partition.on.length; ++i)
			verts[partition.on[i]] = graph.vertex(partition.on[i][0], partition.on[i][1]);

		//Build buckets
		let vis = partition.vis;
		let buckets = [];
		let lastSteiner = null;
		for (var i = 0; i < vis.length;) {
			let v0 = i;
			let v1 = Math.min(i + BUCKET_SIZE - 1, vis.length - 1);
			while (++v1 < vis.length && vis[v1 - 1][1] === vis[v1][1]) {}
			i = v1;
			let bb = makeBucket(vis.slice(v0, v1), x);
			if (lastSteiner && bb.steiner0 &&
				!geom.stabBox(lastSteiner[0], lastSteiner[1], bb.steiner0[0], bb.steiner0[1]))
				edges.push([lastSteiner, bb.steiner0]);

			lastSteiner = bb.steiner1;
			buckets.push(new Bucket(
				bb.y0,
				bb.y1,
				makeVertex(bb.steiner0),
				makeVertex(bb.steiner1),
				bb.left.map(makeVertex),
				bb.right.map(makeVertex),
				bb.on.map(makeVertex)
			));
		}

		return new Node(x, buckets, left, right);
	}
	let root = makeTree(geom.corners, -Infinity, Infinity);

	//Link edges
	for (let i = 0; i < edges.length; ++i)
		graph.link(verts[edges[i][0]], verts[edges[i][1]]);

	//Initialized graph
	graph.init();

	//Return resulting tree
	return new L1PathPlanner(geom, graph, root);
}

export default createPlanner;
