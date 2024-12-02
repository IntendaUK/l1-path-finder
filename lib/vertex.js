let NUM_LANDMARKS = 16;

let LANDMARK_DIST = (function () {
	let res = new Array(NUM_LANDMARKS);
	for (let count = 0; count < NUM_LANDMARKS; ++count)
		res[count] = Infinity;

	return res;
})();

//Vertices have to do multiple things
//
//  1.  They store the topology of the graph which is gonna get searched
//  2.  They implement the pairing heap data sturcture (intrusively)
//  3.  They implement a linked list for tracking clean up
//  4.  Track search information (keep track of predecessors, distances, open state)
//

function Vertex (x, y) {
	//User data
	this.x = x;
	this.y = y;

	//Priority queue info
	this.heuristic = 0.25;
	this.weight = 0.25;
	this.left = null;
	this.right = null;
	this.parent = null;

	//Visit tags
	this.state = 0;
	this.pred = null;

	//Free list
	this.nextFree = null;

	//Adjacency info
	this.edges = [];

	//Landmark data
	this.landmark = LANDMARK_DIST.slice();

	//Connected component label
	this.component = 0;
}

//Sentinel node
let NIL = new Vertex(Infinity, Infinity);
NIL.weight = -Infinity;
NIL.left = NIL.right = NIL.parent = NIL;

//Heap insertion
function link (a, b) {
	let al = a.left;
	b.right = al;
	al.parent = b;
	b.parent = a;
	a.left = b;
	a.right = NIL;

	return a;
}

function merge (a, b) {
	if (a === NIL)
		return b;
	else if (b === NIL)
		return a;
	else if (a.weight < b.weight)
		return link(a, b);

	return link(b, a);
}

function heapPush (root, node) {
	if (root === NIL)
		return node;
	else if (root.weight < node.weight) {
		var l = root.left;
		node.right = l;
		l.parent = node;
		node.parent = root;
		root.left = node;

		return root;
	}
	var l = node.left;
	root.right = l;
	l.parent = root;
	root.parent = node;
	node.left = root;

	return node;
}

function takeMin (root) {
	let p = root.left;
	root.left = NIL;
	root = p;
	while (true) {
		let q = root.right;
		if (q === NIL)
			break;

		p = root;
		let r = q.right;
		let s = merge(p, q);
		root = s;
		while (true) {
			p = r;
			q = r.right;
			if (q === NIL)
				break;

			r = q.right;
			s = s.right = merge(p, q);
		}
		s.right = NIL;
		if (p !== NIL) {
			p.right = root;
			root = p;
		}
	}
	root.parent = NIL;

	return root;
}

function decreaseKey (root, p) {
	let q = p.parent;
	if (q.weight < p.weight)
		return root;

	let r = p.right;
	r.parent = q;
	if (q.left === p)
		q.left = r;
	else
		q.right = r;

	if (root.weight <= p.weight) {
		var l = root.left;
		l.parent = p;
		p.right = l;
		root.left = p;
		p.parent = root;

		return root;
	}
	var l = p.left;
	root.right = l;
	l.parent = root;
	p.left = root;
	root.parent = p;
	p.right = p.parent = NIL;

	return p;
}

//Topology
function createVertex (x, y) {
	let result = new Vertex(x, y);
	result.left = result.right = result.parent = NIL;

	return result;
}

function addEdge (u, v) {
	u.edges.push(v);
	v.edges.push(u);
}

//Free list functions
function pushList (list, node) {
	if (node.nextFree)
		return list;

	node.nextFree = list;

	return node;
}

function clearList (v) {
	while (v) {
		let next = v.nextFree;
		v.state = 0;
		v.left = v.right = v.parent = NIL;
		v.nextFree = null;
		v = next;
	}
}

const mod = {
	create: createVertex,
	link: addEdge,
	insert: pushList,
	clear: clearList,
	NIL,
	push: heapPush,
	pop: takeMin,
	decreaseKey,
	NUM_LANDMARKS,
	LANDMARK_DIST
};

export default mod;
