import vtx from './vertex';
let NIL = vtx.NIL;
let NUM_LANDMARKS = vtx.NUM_LANDMARKS;
let LANDMARK_DIST = vtx.LANDMARK_DIST;

function heuristic (tdist, tx, ty, node) {
	let nx = +node.x;
	let ny = +node.y;
	let pi = Math.abs(nx - tx) + Math.abs(ny - ty);
	let ndist = node.landmark;
	for (let i = 0; i < NUM_LANDMARKS; ++i)
		pi = Math.max(pi, tdist[i] - ndist[i]);

	return 1.0000009536743164 * pi;
}

function Graph () {
	this.target = vtx.create(0, 0);
	this.verts = [];
	this.freeList = this.target;
	this.toVisit = NIL;
	this.lastS = null;
	this.lastT = null;
	this.srcX = 0;
	this.srcY = 0;
	this.dstX = 0;
	this.dstY = 0;
	this.landmarks = [];
	this.landmarkDist = LANDMARK_DIST.slice();
}

let proto = Graph.prototype;

proto.vertex = function (x, y) {
	let v = vtx.create(x, y);
	this.verts.push(v);

	return v;
};

proto.link = function (u, v) {
	vtx.link(u, v);
};

proto.setSourceAndTarget = function (sx, sy, tx, ty) {
	this.srcX = sx | 0;
	this.srcY = sy | 0;
	this.dstX = tx | 0;
	this.dstY = ty | 0;
};

//Mark vertex connected to source
proto.addS = function (v) {
	if ((v.state & 2) === 0) {
		v.heuristic = heuristic(this.landmarkDist, this.dstX, this.dstY, v);
		v.weight = Math.abs(this.srcX - v.x) + Math.abs(this.srcY - v.y) + v.heuristic;
		v.state |= 2;
		v.pred = null;
		this.toVisit = vtx.push(this.toVisit, v);
		this.freeList = vtx.insert(this.freeList, v);
		this.lastS = v;
	}
};

//Mark vertex connected to target
proto.addT = function (v) {
	if ((v.state & 1) === 0) {
		v.state |= 1;
		this.freeList = vtx.insert(this.freeList, v);
		this.lastT = v;

		//Update heuristic
		let d = Math.abs(v.x - this.dstX) + Math.abs(v.y - this.dstY);
		let vdist = v.landmark;
		let tdist = this.landmarkDist;
		for (let i = 0; i < NUM_LANDMARKS; ++i)
			tdist[i] = Math.min(tdist[i], vdist[i] + d);
	}
};

//Retrieves the path from dst->src
proto.getPath = function (out) {
	let prevX = this.dstX;
	let prevY = this.dstY;
	out.push(prevX, prevY);
	let head = this.target.pred;
	while (head) {
		if (prevX !== head.x && prevY !== head.y)
			out.push(head.x, prevY);

		if (prevX !== head.x || prevY !== head.y)
			out.push(head.x, head.y);

		prevX = head.x;
		prevY = head.y;
		head = head.pred;
	}
	if (prevX !== this.srcX && prevY !== this.srcY)
		out.push(this.srcX, prevY);

	if (prevX !== this.srcX || prevY !== this.srcY)
		out.push(this.srcX, this.srcY);

	return out;
};

proto.findComponents = function () {
	let verts = this.verts;
	let n = verts.length;
	for (var i = 0; i < n; ++i)
		verts[i].component = -1;

	let components = [];
	for (var i = 0; i < n; ++i) {
		let root = verts[i];
		if (root.component >= 0)
			continue;

		let label = components.length;
		root.component = label;
		let toVisit = [root];
		let ptr = 0;
		while (ptr < toVisit.length) {
			let v = toVisit[ptr++];
			let adj = v.edges;
			for (let j = 0; j < adj.length; ++j) {
				let u = adj[j];
				if (u.component >= 0)
					continue;

				u.component = label;
				toVisit.push(u);
			}
		}
		components.push(toVisit);
	}

	return components;
};

//Find all landmarks
function compareVert (a, b) {
	let d = a.x - b.x;
	if (d) return d;

	return a.y - b.y;
}

//For each connected component compute a set of landmarks
proto.findLandmarks = function (component) {
	component.sort(compareVert);
	let v = component[component.length >>> 1];
	for (let k = 0; k < NUM_LANDMARKS; ++k) {
		v.weight = 0.0;
		this.landmarks.push(v);
		for (let toVisit = v; toVisit !== NIL;) {
			v = toVisit;
			v.state = 2;
			toVisit = vtx.pop(toVisit);
			let w = v.weight;
			let adj = v.edges;
			for (var i = 0; i < adj.length; ++i) {
				var u = adj[i];
				if (u.state === 2)
					continue;

				let d = w + Math.abs(v.x - u.x) + Math.abs(v.y - u.y);
				if (u.state === 0) {
					u.state = 1;
					u.weight = d;
					toVisit = vtx.push(toVisit, u);
				} else if (d < u.weight) {
					u.weight = d;
					toVisit = vtx.decreaseKey(toVisit, u);
				}
			}
		}
		let farthestD = 0;
		for (var i = 0; i < component.length; ++i) {
			var u = component[i];
			u.state = 0;
			u.landmark[k] = u.weight;
			let s = Infinity;
			for (let j = 0; j <= k; ++j)
				s = Math.min(s, u.landmark[j]);

			if (s > farthestD) {
				v = u;
				farthestD = s;
			}
		}
	}
};

proto.init = function () {
	let components = this.findComponents();
	for (let i = 0; i < components.length; ++i)
		this.findLandmarks(components[i]);
};

//Runs a* on the graph
proto.search = function () {
	let target = this.target;
	let freeList = this.freeList;
	let tdist = this.landmarkDist;

	//Initialize target properties
	let dist = Infinity;

	//Test for case where S and T are disconnected
	if (this.lastS && this.lastT &&
      this.lastS.component === this.lastT.component) {
		let sx = +this.srcX;
		let sy = +this.srcY;
		let tx = +this.dstX;
		let ty = +this.dstY;

		for (let toVisit = this.toVisit; toVisit !== NIL;) {
			let node = toVisit;
			let nx = +node.x;
			let ny = +node.y;
			let d = Math.floor(node.weight - node.heuristic);

			if (node.state === 3) {
				//If node is connected to target, exit
				dist = d + Math.abs(tx - nx) + Math.abs(ty - ny);
				target.pred = node;
				break;
			}

			//Mark node closed
			node.state = 4;

			//Pop node from toVisit queue
			toVisit = vtx.pop(toVisit);

			let adj = node.edges;
			let n = adj.length;
			for (var i = 0; i < n; ++i) {
				let v = adj[i];
				let state = v.state;
				if (state === 4)
					continue;

				let vd = d + Math.abs(nx - v.x) + Math.abs(ny - v.y);
				if (state < 2) {
					let vh = heuristic(tdist, tx, ty, v);
					v.state |= 2;
					v.heuristic = vh;
					v.weight = vh + vd;
					v.pred = node;
					toVisit = vtx.push(toVisit, v);
					freeList = vtx.insert(freeList, v);
				} else {
					let vw = vd + v.heuristic;
					if (vw < v.weight) {
						v.weight = vw;
						v.pred = node;
						toVisit = vtx.decreaseKey(toVisit, v);
					}
				}
			}
		}
	}

	//Clear the free list & priority queue
	vtx.clear(freeList);

	//Reset pointers
	this.freeList = target;
	this.toVisit = NIL;
	this.lastS = this.lastT = null;

	//Reset landmark distance
	for (var i = 0; i < NUM_LANDMARKS; ++i)
		tdist[i] = Infinity;

	//Return target distance
	return dist;
};

export default Graph;
