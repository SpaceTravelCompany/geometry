#+private
package geometry

import "base:intrinsics"
import "base:runtime"
import "core:math"
import "core:math/linalg"
import "core:mem"
import "core:slice"

import "core:math/fixed"

import utils "shared:utils_private"
import "shared:utils_private/fixed_ex"

Edge :: struct {
	vL:       ^Vertex,
	vR:       ^Vertex,
	vB:       ^Vertex,
	vT:       ^Vertex,
	triA:     ^Triangle,
	triB:     ^Triangle,
	nextE:    ^Edge,
	prevE:    ^Edge,
	kind:     EdgeKind,
	isActive: bool,
}

Triangle :: struct {
	edges: [3]^Edge,
}

EdgeKind :: enum u8 {
	loose,
	ascend,
	descend,
} // ascend & descend are 'fixed' edges

EdgeContainsResult :: enum u8 {
	neither,
	left,
	right,
}

Vertex :: struct {
	p:       [2]FixedDef,
	e:       [dynamic]^Edge,
	innerLM: bool, //check inner local minimum
}

Context :: struct {
	all_verts:            [dynamic]Vertex,
	loc_min_stack:        [dynamic]^Vertex,
	horz_edge_stack:      [dynamic]^Edge,
	all_edges:            [dynamic]^Edge,
	all_tris:             [dynamic]^Triangle,
	pendingDelaunayStack: [dynamic]^Edge,
	indices:              [dynamic]u32,
	polys:                [][][2]FixedDef,
	polyCCW:              []PolyOrientation,
	lowermostVertex:      ^Vertex,
	firstActive:          ^Edge,
}

HorizontalBetween :: proc(ctx: ^Context, v1, v2: ^Vertex) -> ^Edge {
	y := v1.p.y.i
	l, r: FixedDef
	if v1.p.x.i > v2.p.x.i {
		l = v2.p.x
		r = v1.p.x
	} else {
		l = v1.p.x
		r = v2.p.x
	}

	res := ctx.firstActive
	for res != nil {
		if res.vL.p.y.i == y &&
		   res.vR.p.y.i == y &&
		   res.vL.p.x.i >= l.i &&
		   res.vR.p.x.i <= r.i &&
		   (res.vL.p.x.i != l.i || res.vL.p.x.i != r.i) {
			break
		}
		res = res.nextE
	}
	return res
}


EdgeContains :: proc "contextless" (edge: ^Edge, v: ^Vertex) -> EdgeContainsResult {
	if edge.vL == v do return .left
	if edge.vR == v do return .right
	return .neither
}

RemoveEdgeFromVertex :: proc(vert: ^Vertex, edge: ^Edge) {
	for e, i in vert.e {
		if e == edge {
			ordered_remove(&vert.e, i)
			return
		}
	}
	panic("edge not found in vertex")
}


IsHorizontal :: proc "contextless" (e: ^Edge) -> bool {
	return e.vB.p.y.i == e.vT.p.y.i
}

MakeVertex :: proc(p: [2]FixedDef) -> (res:Vertex, err:Geometry_Error) {
	res = Vertex {
		p       = p,
		innerLM = false,
	}
	res.e = make([dynamic]^Edge, context.temp_allocator) or_return
	non_zero_reserve(&res.e, 2) or_return
	return
}

MakeEdge :: proc(
	ctx: ^Context,
	v1, v2: ^Vertex,
	kind: EdgeKind,
) -> (
	res: ^Edge,
	err: Geometry_Error,
) {
	non_zero_append(&ctx.all_edges, new(Edge, context.temp_allocator) or_return) or_return
	if len(&ctx.all_edges) >= int(max(u32) - 1) do return nil, .TOO_MANY_EDGES // - 1 because max(u32) uses nil

	ed := ctx.all_edges[len(ctx.all_edges) - 1]

	if v1.p.y.i == v2.p.y.i {
		ed.vB = v1; ed.vT = v2
	} else if v1.p.y.i < v2.p.y.i {
		ed.vB = v2; ed.vT = v1
	} else {
		ed.vB = v1; ed.vT = v2
	}

	if v1.p.x.i <= v2.p.x.i {
		ed.vL = v1; ed.vR = v2
	} else {
		ed.vL = v2; ed.vR = v1
	}

	ed.kind = kind
	non_zero_append(&v1.e, ed) or_return
	non_zero_append(&v2.e, ed) or_return

	if kind == .loose {
		non_zero_append(&ctx.pendingDelaunayStack, ed) or_return
		SetEdgeToActive(ctx, ed)
	}

	return ed, nil
}

SetEdgeToActive :: proc "contextless" (ctx: ^Context, edge: ^Edge) {
	// nb: on occassions this method can get called twice for a given edge
	// This is because, in the Triangulate() method where vertex 'edges'
	// arrays are being parsed, edges can can be removed from the array
	// which changes the index of following edges.
	if edge.isActive do return

	edge.prevE = nil
	edge.nextE = ctx.firstActive
	edge.isActive = true
	if ctx.firstActive != nil {
		ctx.firstActive.prevE = edge
	}
	ctx.firstActive = edge
}

RemoveEdgeFromActives :: proc (ctx: ^Context, edge: ^Edge) {
	RemoveEdgeFromVertex(edge.vB, edge)
	RemoveEdgeFromVertex(edge.vT, edge)
	prev := edge.prevE
	next := edge.nextE
	if next != nil do next.prevE = prev
	if prev != nil do prev.nextE = next
	edge.isActive = false
	if ctx.firstActive == edge do ctx.firstActive = next
}

SplitEdge :: proc(ctx: ^Context, longE, shortE: ^Edge) -> (err: Geometry_Error) {
	oldT := longE.vT
	newT := shortE.vT
	// remove longEdge from longEdge.vT.edges
	RemoveEdgeFromVertex(oldT, longE)
	// shorten longEdge
	longE.vT = newT
	if longE.vL == oldT {
		longE.vL = newT
	} else {
		longE.vR = newT
	}
	// add shortened longEdge to newT.edges
	non_zero_append(&newT.e, longE) or_return
	// and create a new edge betweem newV, oldT
	MakeEdge(ctx, newT, oldT, longE.kind) or_return
	return
}


RemoveIntersection :: proc(
	ctx: ^Context,
	e1: ^Edge,
	e2: ^Edge,
) -> (
	res: bool,
	err: Geometry_Error,
) {
	// find which vertex is closest to the other segment
	// (ie not the vertex closest to the intersection point)
	v: ^Vertex = e1.vL
	tmpE: ^Edge = e2
	d, d_ := ShortestLength2Line(e1.vL.p, e2.vL.p, e2.vR.p)
	d2, d2_ := ShortestLength2Line(e1.vR.p, e2.vL.p, e2.vR.p)
	if fixed.mul(d2, d_).i < fixed.mul(d, d2_).i { 	// cross mul compare d2 / d2_ < d / d_
		d = d2; d_ = d2_
		v = e1.vR
	}
	d2, d2_ = ShortestLength2Line(e2.vL.p, e1.vL.p, e1.vR.p)
	if fixed.mul(d2, d_).i < fixed.mul(d, d2_).i {
		d = d2; d_ = d2_
		tmpE = e1
		v = e2.vL
	}
	d2, d2_ = ShortestLength2Line(e2.vR.p, e1.vL.p, e1.vR.p)
	if fixed.mul(d2, d_).i < fixed.mul(d, d2_).i {
		d = d2; d_ = d2_
		tmpE = e1
		v = e2.vR
	}
	if d.i > d_.i { 	//d.i > 1 << FIXED_SHIFT {
		return false, nil // not just a simple 'rounding' intersection
	}
	// split 'tmpE' into 2 edges at 'v'
	v2 := tmpE.vT
	RemoveEdgeFromVertex(v2, tmpE)
	// replace v2 in tmpE with v
	if tmpE.vL == v2 {
		tmpE.vL = v
	} else {
		tmpE.vR = v
	}
	tmpE.vT = v
	non_zero_append(&v.e, tmpE) or_return
	v.innerLM = false // #47
	// left turning is angle positive
	if tmpE.vB.innerLM && GetLocMinAngle(tmpE.vB) <= 0 {
		tmpE.vB.innerLM = false // #44, 52
	}
	// finally create a new edge between v and v2
	MakeEdge(ctx, v, v2, tmpE.kind) or_return
	return true, nil
}


GetLocMinAngle :: proc(v: ^Vertex) -> f64 {
	// TODO - recheck the result's sign compared to left vs right turning
	// (currently assumes left turning => positive values)
	// precondition - this function is called before processing locMin.
	//assert(len(v.e) == 2)
	asc, des: int
	if v.e[0].kind == .ascend {
		asc = 0
		des = 1
	} else {
		des = 0
		asc = 1
	}
	// winding direction - descending to ascending
	return GetAngle(
		[2]f64{fixed.to_f64(v.e[des].vT.p.x), fixed.to_f64(v.e[des].vT.p.y)},
		[2]f64{fixed.to_f64(v.p.x), fixed.to_f64(v.p.y)},
		[2]f64{fixed.to_f64(v.e[asc].vT.p.x), fixed.to_f64(v.e[asc].vT.p.y)},
	)
}

vertex_index :: proc "contextless" (ctx: ^Context, v: ^Vertex) -> u32 {
	for i in 0 ..< len(ctx.all_verts) {
		if &ctx.all_verts[i] == v do return u32(i)
	}
	return 0
}

EdgeCompleted :: proc "contextless" (e: ^Edge) -> bool {
	if e.triA == nil do return false
	if e.triB != nil do return true
	return e.kind != .loose
}

IsLooseEdge :: proc "contextless" (e: ^Edge) -> bool {
	return e.kind == .loose
}

// Clipper2: left edge = ascend (fills on right side of edge)
IsLeftEdge :: proc "contextless" (e: ^Edge) -> bool {
	return e.kind == .ascend
}

// Clipper2: right edge = descend
IsRightEdge :: proc "contextless" (e: ^Edge) -> bool {
	return e.kind == .descend
}

FindLinkingEdge :: proc "contextless" (vert1, vert2: ^Vertex, prefer_ascending: bool) -> ^Edge {
	res: ^Edge = nil
	for e in vert1.e {
		if e.vL == vert2 || e.vR == vert2 {
			if e.kind == .loose ||
			   (e.kind == .ascend) == prefer_ascending {
				return e
			}
			res = e
		}
	}
	return res
}