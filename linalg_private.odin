#+private
package geometry

import "base:intrinsics"
import "base:runtime"
import "core:math"
import "core:math/linalg"
import "core:mem"
import "core:slice"

import utils "shared:utils_private"
import "shared:utils_private/fixed_bcd"

EdgeKind :: enum u8 {
	loose,
	ascend,
	descend,
}

EdgeContainsResult :: enum u8 {
	neither,
	left,
	right,
}

Vertex :: struct($T: typeid) where intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	p:       [2]T,
	e:       [dynamic]^Edge(T),
	innerLM: bool,
}

Edge :: struct($T: typeid) where intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	vL:       ^Vertex(T),
	vR:       ^Vertex(T),
	vB:       ^Vertex(T),
	vT:       ^Vertex(T),
	triA:     ^Triangle(T),
	triB:     ^Triangle(T),
	nextE:    ^Edge(T),
	prevE:    ^Edge(T),
	kind:     EdgeKind,
	isActive: bool,
}

Triangle :: struct($T: typeid) where intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	edges: [3]^Edge(T),
}

Context :: struct($T: typeid) where intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	all_verts:            [dynamic]^Vertex(T),
	loc_min_stack:        [dynamic]^Vertex(T),
	horz_edge_stack:      [dynamic]^Edge(T),
	all_edges:            [dynamic]^Edge(T),
	all_tris:             [dynamic]^Triangle(T),
	pendingDelaunayStack: [dynamic]^Edge(T),
	indices:              [dynamic]u32,
	polys:                [][][2]T,
	lowermostVertex:      ^Vertex(T),
	firstActive:          ^Edge(T),
}

HorizontalBetween :: proc(ctx: ^Context($T), v1, v2: ^Vertex(T)) -> ^Edge(T) {
	y := v1.p.y.i
	l, r: T
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

EdgeContains :: proc "contextless" (edge: ^Edge($T), v: ^Vertex(T)) -> EdgeContainsResult {
	if edge.vL == v do return .left
	if edge.vR == v do return .right
	return .neither
}

RemoveEdgeFromVertex :: proc(vert: ^Vertex($T), edge: ^Edge(T)) -> (err: Geometry_Error) {
	for e, i in vert.e {
		if e == edge {
			ordered_remove(&vert.e, i)
			return nil
		}
	}
	return .NO_EDGE_IN_VERTEX
}

IsHorizontal :: proc "contextless" (e: ^Edge($T)) -> bool {
	return e.vB.p.y.i == e.vT.p.y.i
}

MakeVertex :: proc(
	p: [2]$T,
) -> (
	res: ^Vertex(T),
	err: Geometry_Error,
) where intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	res = new_clone(Vertex(T){p = p, innerLM = false}, context.temp_allocator) or_return
	res.e = make([dynamic]^Edge(T), context.temp_allocator) or_return
	non_zero_reserve(&res.e, 2) or_return
	return
}

MakeEdge :: proc(
	ctx: ^Context($T),
	v1, v2: ^Vertex(T),
	kind: EdgeKind,
) -> (
	res: ^Edge(T),
	err: Geometry_Error,
) where intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	non_zero_append(&ctx.all_edges, new(Edge(T), context.temp_allocator) or_return) or_return
	if len(ctx.all_edges) >= int(max(u32) - 1) do return nil, .TOO_MANY_EDGES

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

SetEdgeToActive :: proc "contextless" (ctx: ^Context($T), edge: ^Edge(T)) {
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

RemoveEdgeFromActives :: proc(ctx: ^Context($T), edge: ^Edge(T)) -> (err: Geometry_Error) {
	RemoveEdgeFromVertex(edge.vB, edge) or_return
	if edge.vT != edge.vB do RemoveEdgeFromVertex(edge.vT, edge) or_return
	prev := edge.prevE
	next := edge.nextE
	if next != nil do next.prevE = prev
	if prev != nil do prev.nextE = next
	edge.isActive = false
	if ctx.firstActive == edge do ctx.firstActive = next
	return nil
}

SplitEdge :: proc(ctx: ^Context($T), longE, shortE: ^Edge(T)) -> (err: Geometry_Error) {
	oldT := longE.vT
	newT := shortE.vT
	RemoveEdgeFromVertex(oldT, longE) or_return
	longE.vT = newT
	if longE.vL == oldT do longE.vL = newT
	else do longE.vR = newT
	non_zero_append(&newT.e, longE) or_return
	MakeEdge(ctx, newT, oldT, longE.kind) or_return
	return
}

RemoveIntersection :: proc(
	ctx: ^Context($T),
	e1: ^Edge(T),
	e2: ^Edge(T),
) -> (
	res: bool,
	err: Geometry_Error,
) where intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	v: ^Vertex(T) = e1.vL
	tmpE: ^Edge(T) = e2
	d, d_ := ShortestLength2Line(e1.vL.p, e2.vL.p, e2.vR.p)
	d2, d2_ := ShortestLength2Line(e1.vR.p, e2.vL.p, e2.vR.p)
	if fixed_bcd.mul(d2, d_).i < fixed_bcd.mul(d, d2_).i {
		d = d2; d_ = d2_
		v = e1.vR
	}
	d2, d2_ = ShortestLength2Line(e2.vL.p, e1.vL.p, e1.vR.p)
	if fixed_bcd.mul(d2, d_).i < fixed_bcd.mul(d, d2_).i {
		d = d2; d_ = d2_
		tmpE = e1
		v = e2.vL
	}
	d2, d2_ = ShortestLength2Line(e2.vR.p, e1.vL.p, e1.vR.p)
	if fixed_bcd.mul(d2, d_).i < fixed_bcd.mul(d, d2_).i {
		d = d2; d_ = d2_
		tmpE = e1
		v = e2.vR
	}
	if d.i > d_.i do return false, nil
	v2 := tmpE.vT
	RemoveEdgeFromVertex(v2, tmpE) or_return
	if tmpE.vL == v2 do tmpE.vL = v
	else do tmpE.vR = v
	tmpE.vT = v
	non_zero_append(&v.e, tmpE) or_return
	v.innerLM = false
	if tmpE.vB.innerLM && GetLocMinAngle(tmpE.vB) <= 0 do tmpE.vB.innerLM = false
	MakeEdge(ctx, v, v2, tmpE.kind) or_return
	return true, nil
}

GetLocMinAngle :: proc(
	v: ^Vertex($T),
) -> f64 where intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	asc, des: int
	if v.e[0].kind == .ascend {asc = 0; des = 1} else {des = 0; asc = 1}
	return GetAngle(
		[2]f64{fixed_bcd.to_f64(v.e[des].vT.p.x), fixed_bcd.to_f64(v.e[des].vT.p.y)},
		[2]f64{fixed_bcd.to_f64(v.p.x), fixed_bcd.to_f64(v.p.y)},
		[2]f64{fixed_bcd.to_f64(v.e[asc].vT.p.x), fixed_bcd.to_f64(v.e[asc].vT.p.y)},
	)
}

vertex_index :: proc "contextless" (ctx: ^Context($T), v: ^Vertex(T)) -> u32 {
	for i in 0 ..< len(ctx.all_verts) {
		if ctx.all_verts[i] == v do return u32(i)
	}
	return 0
}

EdgeCompleted :: proc "contextless" (e: ^Edge($T)) -> bool {
	if e.triA == nil do return false
	if e.triB != nil do return true
	return e.kind != .loose
}

IsLooseEdge :: proc "contextless" (e: ^Edge($T)) -> bool {
	return e.kind == .loose
}

IsLeftEdge :: proc "contextless" (e: ^Edge($T)) -> bool {
	return e.kind == .ascend
}

IsRightEdge :: proc "contextless" (e: ^Edge($T)) -> bool {
	return e.kind == .descend
}

FindLinkingEdge :: proc "contextless" (
	vert1, vert2: ^Vertex($T),
	prefer_ascending: bool,
) -> ^Edge(T) {
	res: ^Edge(T) = nil
	for e in vert1.e {
		if e.vL == vert2 || e.vR == vert2 {
			if e.kind == .loose || (e.kind == .ascend) == prefer_ascending do return e
			res = e
		}
	}
	return res
}

