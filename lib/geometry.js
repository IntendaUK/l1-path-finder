/* eslint-disable max-lines-per-function, func-style, complexity, no-var, max-lines, block-scoped-var, max-len, no-return-assign */

import ndarray from 'ndarray';
import ops from 'ndarray-ops';
import prefixSum from 'ndarray-prefix-sum';
import _orient from 'robust-orientation';

const orient = _orient[3];

function Geometry (corners, grid) {
	this.corners = corners;
	this.grid = grid;
}

let proto = Geometry.prototype;

proto.stabRay = function (vx, vy, x) {
	return this.stabBox(vx, vy, x, vy);
};

proto.stabTile = function (x, y) {
	return this.stabBox(x, y, x, y);
};

proto.integrate = function (x, y) {
	if (x < 0 || y < 0)
		return 0;

	return this.grid.get(
		Math.min(x, this.grid.shape[0] - 1) | 0,
		Math.min(y, this.grid.shape[1] - 1) | 0);
};

proto.stabBox = function (ax, ay, bx, by) {
	let lox = Math.min(ax, bx);
	let loy = Math.min(ay, by);
	let hix = Math.max(ax, bx);
	let hiy = Math.max(ay, by);

	let s = this.integrate(lox - 1, loy - 1)
        - this.integrate(lox - 1, hiy)
        - this.integrate(hix, loy - 1)
        + this.integrate(hix, hiy);

	return s > 0;
};

function createGeometry (grid, loops) {
	//Extract corners
	let corners = [];
	for (let k = 0; k < loops.length; ++k) {
		let polygon = loops[k];
		for (let i = 0; i < polygon.length; ++i) {
			let a = polygon[(i + polygon.length - 1) % polygon.length];
			let b = polygon[i];
			let c = polygon[(i + 1) % polygon.length];
			if (orient(a, b, c) > 0) {
				let offset = [0, 0];
				for (let j = 0; j < 2; ++j) {
					if (b[j] - a[j])
						offset[j] = b[j] - a[j];
					else
						offset[j] = b[j] - c[j];

					offset[j] = b[j] + Math.min(Math.round(offset[j] / Math.abs(offset[j])) | 0, 0);
				}
				corners.push(offset);
			}
		}
	}

	//Create integral image
	prefixSum(grid);

	//Return resulting geometry
	return new Geometry(corners, grid);
}

export default createGeometry;
