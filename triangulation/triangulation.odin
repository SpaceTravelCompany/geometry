package triangulation

import "../linalg_ex"
import "base:intrinsics"
import "base:runtime"
import "core:math"
import "core:math/linalg"
import "core:mem"
import "core:slice"

import "core:math/fixed"
import utils "shared:utils_private"

FixedDef :: fixed.Fixed(i64, 38)

__Trianguate_Error :: enum {
	TOO_MANY_EDGES,
	NO_PATHS,
	NO_EDGE_IN_VERTEX,
	FIRST_ACTIVE_MISSING,
	EBELLOW_MISSING,
	PATHS_INTERSECTS,
	FORCELEGAL_EDGEA_NIL,
	FORCELEGAL_EDGEB_NIL,
	FORCELEGAL_UNKNOWN1,
	FORCELEGAL_UNKNOWN2,
}

Trianguate_Error :: union #shared_nil {
	__Trianguate_Error,
	runtime.Allocator_Error,
}

@(private = "file")
Context :: struct {
	all_verts:            [dynamic]^Vertex,
	loc_min_stack:        [dynamic]^Vertex,
	horz_edge_stack:      [dynamic]^Edge,
	all_edges:            [dynamic]^Edge,
	all_tris:             [dynamic]^Triangle,
	pendingDelaunayStack: [dynamic]^Edge,
	indices:              [dynamic]u32,
	polys:                [][][2]FixedDef,
	lowermostVertex:      ^Vertex,
	firstActive:          ^Edge,
}


@(private = "file")
EdgeKind :: enum u8 {
	loose,
	ascend,
	descend,
}

@(private = "file")
EdgeContainsResult :: enum u8 {
	neither,
	left,
	right,
}

@(private = "file")
Vertex :: struct {
	p:       [2]FixedDef,
	e:       [dynamic]^Edge,
	innerLM: bool,
}

@(private = "file")
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

@(private = "file")
Triangle :: struct {
	edges: [3]^Edge,
}

@(private = "file")
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
		   (res.vL.p.x.i != l.i || res.vL.p.x.i != r.i) { 	// TODO vR로 바꿔본다.
			break
		}
		res = res.nextE
	}
	return res
}

// Precondition: ctx.all_edges must be sorted ascending on edge.vL.p.x
@(private = "file")
FixupEdgeIntersects :: proc(ctx: ^Context) -> (err: Trianguate_Error) {
	for i1 in 0 ..< len(ctx.all_edges) {
		e1 := ctx.all_edges[i1]
		for i2 in i1 + 1 ..< len(ctx.all_edges) {
			e2 := ctx.all_edges[i2]
			if e2.vL.p.x.i >= e1.vR.p.x.i {
				break
			}
			if e2.vT.p.y.i < e1.vB.p.y.i && e2.vB.p.y.i > e1.vT.p.y.i {
				if linalg_ex.LinesIntersect3(e2.vL.p, e2.vR.p, e1.vL.p, e1.vR.p, true) ==
				   .intersect {
					RemoveIntersection(ctx, e2, e1) or_return
				}
			}
		}
	}
	return nil
}

@(private = "file")
TrianguatePolygons_Fixed_Impl :: proc(
	poly: [][][2]FixedDef,
	allocator := context.allocator,
) -> (
	indices: []u32,
	err: Trianguate_Error,
) {
	ctx := Context {
		all_verts            = make([dynamic]^Vertex, context.temp_allocator) or_return,
		all_tris             = make([dynamic]^Triangle, context.temp_allocator) or_return,
		all_edges            = make([dynamic]^Edge, context.temp_allocator) or_return,
		pendingDelaunayStack = make([dynamic]^Edge, context.temp_allocator) or_return,
		loc_min_stack        = make([dynamic]^Vertex, context.temp_allocator) or_return,
		horz_edge_stack      = make([dynamic]^Edge, context.temp_allocator) or_return,
		polys                = poly,
	}
	non_zero_reserve(&ctx.all_edges, len(poly)) or_return

	AddPaths(&ctx) or_return

	if ctx.lowermostVertex.innerLM {
		lm: ^Vertex
		for len(ctx.loc_min_stack) > 0 {
			lm = ctx.loc_min_stack[len(ctx.loc_min_stack) - 1]
			lm.innerLM = !lm.innerLM
			non_zero_resize(&ctx.loc_min_stack, len(ctx.loc_min_stack) - 1) or_return
		}

		for &e in ctx.all_edges {
			if e.kind == .ascend do e.kind = .descend
			else do e.kind = .ascend
		}
	} else {
		clear(&ctx.loc_min_stack)
	}

	slice.sort_by(ctx.all_edges[:], proc(a, b: ^Edge) -> bool {
		return a.vL.p.x.i < b.vL.p.x.i
	})
	FixupEdgeIntersects(&ctx) or_return

	slice.sort_by(ctx.all_verts[:], proc(a, b: ^Vertex) -> bool {
		if a.p.y.i == b.p.y.i do return a.p.x.i < b.p.x.i
		return a.p.y.i > b.p.y.i
	})
	MergeDupOrCollinearVertices(&ctx) or_return

	currY: FixedDef = ctx.all_verts[0].p.y
	for &v in ctx.all_verts {
		if len(v.e) == 0 do continue

		if v.p.y.i != currY.i {
			// JOIN AN INNER LOCMIN WITH A SUITABLE EDGE BELOW
			for len(ctx.loc_min_stack) > 0 {
				lm := ctx.loc_min_stack[len(ctx.loc_min_stack) - 1]
				non_zero_resize(&ctx.loc_min_stack, len(ctx.loc_min_stack) - 1) or_return

				e := CreateInnerLocMinLooseEdge(&ctx, lm) or_return

				if IsHorizontal(e) {
					if e.vL == e.vB {
						DoTriangulateLeft(&ctx, e, e.vB, currY) or_return
					} else {
						DoTriangulateRight(&ctx, e, e.vB, currY) or_return
					}
				} else {
					DoTriangulateLeft(&ctx, e, e.vB, currY) or_return
					if !EdgeCompleted(e) do DoTriangulateRight(&ctx, e, e.vB, currY) or_return
				}

				// and because adding locMin edges to Actives was delayed ..
				SetEdgeToActive(&ctx, lm.e[0])
				SetEdgeToActive(&ctx, lm.e[1])
			}

			for len(ctx.horz_edge_stack) > 0 {
				e := ctx.horz_edge_stack[len(ctx.horz_edge_stack) - 1]
				non_zero_resize(&ctx.horz_edge_stack, len(ctx.horz_edge_stack) - 1)
				if EdgeCompleted(e) do continue

				if e.vB == e.vL { 	// #45
					if IsLeftEdge(e) {
						DoTriangulateLeft(&ctx, e, e.vB, currY) or_return
					}
				} else {
					if IsRightEdge(e) {
						DoTriangulateRight(&ctx, e, e.vB, currY) or_return
					}
				}
			}
			currY = v.p.y
		}

		// inner edge loop (내림차순 인덱스 — 루프 중 edges 추가/삭제에 안전)
		for i := len(v.e) - 1; i >= 0; i -= 1 {
			if i >= len(v.e) do continue // v.e 는 루프 중에 변경될 수 있음
			e := v.e[i]
			if EdgeCompleted(e) || IsLooseEdge(e) do continue

			if v == e.vB {
				if IsHorizontal(e) {
					non_zero_append(&ctx.horz_edge_stack, e) or_return
				}
				// locMin 엣지는 actives 추가를 지연
				if !v.innerLM {
					SetEdgeToActive(&ctx, e)
				}
			} else {
				if IsHorizontal(e) {
					non_zero_append(&ctx.horz_edge_stack, e) or_return
				} else if IsLeftEdge(e) {
					DoTriangulateLeft(&ctx, e, e.vB, v.p.y) or_return
				} else {
					DoTriangulateRight(&ctx, e, e.vB, v.p.y) or_return
				}
			}
		} // inner edge loop

		if v.innerLM {
			non_zero_append(&ctx.loc_min_stack, v) or_return
		}
	} // for all_verts

	// 버텍스 루프 후 남은 수평 엣지 처리
	for len(ctx.horz_edge_stack) > 0 {
		e := ctx.horz_edge_stack[len(ctx.horz_edge_stack) - 1]
		non_zero_resize(&ctx.horz_edge_stack, len(ctx.horz_edge_stack) - 1) or_return

		if !EdgeCompleted(e) && e.vB == e.vL {
			DoTriangulateLeft(&ctx, e, e.vB, currY) or_return
		}
	}

	// Delaunay 적합 삼각형으로 변환
	for len(ctx.pendingDelaunayStack) > 0 {
		e := ctx.pendingDelaunayStack[len(ctx.pendingDelaunayStack) - 1]
		non_zero_resize(&ctx.pendingDelaunayStack, len(ctx.pendingDelaunayStack) - 1) or_return
		ForceLegal(&ctx, e) or_return
	}

	// Convert triangles to index buffer; skip degenerate, ensure CW winding
	idx_buf := make([dynamic]u32, context.temp_allocator) or_return
	defer delete(idx_buf)
	for tri in ctx.all_tris {
		vs := triangle_vertices(tri)
		p0, p1, p2 := vs[0].p, vs[1].p, vs[2].p
		cps := linalg_ex.CrossProductSign(p0, p1, p2)
		if cps == 0 do continue // skip degenerate triangles
		i0 := vertex_index(poly, vs[0])
		i1 := vertex_index(poly, vs[1])
		i2 := vertex_index(poly, vs[2])
		if cps < 0 {
			// CCW -> reverse to CW
			i1, i2 = i2, i1
		}
		non_zero_append(&idx_buf, i0, i1, i2) or_return
	}
	indices = utils.make_non_zeroed_slice([]u32, len(idx_buf), allocator) or_return
	mem.copy_non_overlapping(raw_data(indices), raw_data(idx_buf[:]), size_of(u32) * len(idx_buf))
	return
}

@(private = "file")
TrianguatePolygons_WithFrac_Impl :: proc(
	$FRAC: int,
	poly: [][][2]FixedDef,
	allocator := context.allocator,
) -> (
	indices: []u32,
	err: Trianguate_Error,
) where intrinsics.type_is_float {
	poly_fixed := make([dynamic][][2]fixed.BCD(FRAC), context.temp_allocator) or_return
	non_zero_resize(&poly_fixed, len(poly)) or_return

	for i in 0 ..< len(poly) {
		path_fixed := make([dynamic][2]fixed.BCD(FRAC), context.temp_allocator) or_return
		non_zero_resize(&path_fixed, len(poly[i])) or_return

		for j in 0 ..< len(poly[i]) {
			path_fixed[j] = [2]fixed.BCD(FRAC) {
				fixed.from_f64(FRAC, f64(poly[i][j].x)),
				fixed.from_f64(FRAC, f64(poly[i][j].y)),
			}
		}
		poly_fixed[i] = path_fixed[:]
	}

	return TrianguatePolygons_Fixed_Impl(poly_fixed[:], allocator)
}

@(private = "file")
TrianguatePolygons_Impl :: proc(
	poly: [][][2]$T,
	allocator := context.allocator,
) -> (
	indices: []u32,
	err: Trianguate_Error,
) where intrinsics.type_is_float(T) {
	return TrianguatePolygons_WithFrac_Impl(fixed.MAX_FRAC_DIGITS, poly, allocator)
}

@(private = "file")
triangle_vertices :: proc "contextless" (tri: ^Triangle) -> [3]^Vertex {
	e0, e1 := tri.edges[0], tri.edges[1]
	vs: [3]^Vertex
	vs[0] = e0.vL
	vs[1] = e0.vR
	if e1.vL == vs[0] || e1.vL == vs[1] do vs[2] = e1.vR
	else do vs[2] = e1.vL
	return vs
}

@(private = "file")
DoTriangulateLeft :: proc(
	ctx: ^Context,
	edge: ^Edge,
	pivot: ^Vertex,
	minY: FixedDef,
) -> (
	err: Trianguate_Error,
) {
	vAlt: ^Vertex = nil
	eAlt: ^Edge = nil
	v := edge.vB == pivot ? edge.vT : edge.vB

	for e in pivot.e {
		if e == edge || !e.isActive do continue
		vX := e.vT == pivot ? e.vB : e.vT
		if vX == v do continue

		cps := linalg_ex.CrossProductSign(v.p, pivot.p, vX.p)
		if cps == 0 {
			// if pivot is between v and vX then continue
			if (v.p.x.i > pivot.p.x.i) == (pivot.p.x.i > vX.p.x.i) do continue
		} else if cps > 0 ||
		   (vAlt != nil && !(linalg_ex.CrossProductSign(vX.p, pivot.p, vAlt.p) < 0)) {
			continue
		}

		vAlt = vX
		eAlt = e
	}

	if vAlt == nil || vAlt.p.y.i < minY.i do return
	if vAlt.p.y.i < pivot.p.y.i {
		if IsLeftEdge(eAlt) do return
	} else if vAlt.p.y.i > pivot.p.y.i {
		if IsRightEdge(eAlt) do return
	}

	eX := FindLinkingEdge(vAlt, v, vAlt.p.y.i < v.p.y.i)
	if eX == nil {
		if vAlt.p.y.i == v.p.y.i && v.p.y.i == minY.i && HorizontalBetween(ctx, vAlt, v) != nil {
			return
		}
		eX = MakeEdge(ctx, vAlt, v, .loose) or_return
	}
	CreateTriangle(ctx, edge, eAlt, eX) or_return
	if !EdgeCompleted(eX) do DoTriangulateLeft(ctx, eX, vAlt, minY) or_return
	return
}

@(private = "file")
DoTriangulateRight :: proc(
	ctx: ^Context,
	edge: ^Edge,
	pivot: ^Vertex,
	minY: FixedDef,
) -> (
	err: Trianguate_Error,
) {
	vAlt: ^Vertex = nil
	eAlt: ^Edge = nil
	v := edge.vB == pivot ? edge.vT : edge.vB

	for e in pivot.e {
		if e == edge || !e.isActive do continue
		vX := e.vT == pivot ? e.vB : e.vT
		if vX == v do continue

		cps := linalg_ex.CrossProductSign(v.p, pivot.p, vX.p)
		if cps == 0 {
			// if pivot is between v and vX then continue;
			// nb: this is important for both horiz and non-horiz collinear
			if (v.p.x.i > pivot.p.x.i) == (pivot.p.x.i > vX.p.x.i) do continue
		} else if cps < 0 ||
		   (vAlt != nil && !(linalg_ex.CrossProductSign(vX.p, pivot.p, vAlt.p) > 0)) { 	// else if right-turning or not the best edge, then continue
			continue
		}
		vAlt = vX
		eAlt = e
	}
	if vAlt == nil || vAlt.p.y.i < minY.i do return

	if vAlt.p.y.i < pivot.p.y.i {
		if IsRightEdge(eAlt) do return
	} else if vAlt.p.y.i > pivot.p.y.i {
		if IsLeftEdge(eAlt) do return
	}

	eX := FindLinkingEdge(vAlt, v, vAlt.p.y.i > v.p.y.i)
	if eX == nil {
		if vAlt.p.y.i == v.p.y.i && v.p.y.i == minY.i && HorizontalBetween(ctx, vAlt, v) != nil {
			return
		}
		eX = MakeEdge(ctx, vAlt, v, .loose) or_return
	}

	CreateTriangle(ctx, edge, eX, eAlt) or_return
	if !EdgeCompleted(eX) do DoTriangulateRight(ctx, eX, vAlt, minY) or_return

	return
}

@(private = "file")
ForceLegal :: proc(ctx: ^Context, edge: ^Edge) -> (err: Trianguate_Error) {
	if edge.triA == nil || edge.triB == nil do return

	vertA: ^Vertex = nil
	vertB: ^Vertex = nil
	edgesA: [3]^Edge = {nil, nil, nil}
	edgesB: [3]^Edge = {nil, nil, nil}

	for i in 0 ..< 3 {
		if edge.triA.edges[i] == edge do continue
		switch EdgeContains(edge.triA.edges[i], edge.vL) {
		case .left:
			edgesA[1] = edge.triA.edges[i]
			vertA = edge.triA.edges[i].vR
		case .right:
			edgesA[1] = edge.triA.edges[i]
			vertA = edge.triA.edges[i].vL
		case .neither:
			edgesB[1] = edge.triA.edges[i]
		}
	}

	for i in 0 ..< 3 {
		if edge.triB.edges[i] == edge do continue
		switch EdgeContains(edge.triB.edges[i], edge.vL) {
		case .left:
			edgesA[2] = edge.triB.edges[i]
			vertB = edge.triB.edges[i].vR
		case .right:
			edgesA[2] = edge.triB.edges[i]
			vertB = edge.triB.edges[i].vL
		case .neither:
			edgesB[2] = edge.triB.edges[i]
		}
	}

	// InCircleTest reqires edge.triangleA to be a valid triangle
	// if IsEmptyTriangle(edge.triangleA) then Exit; // slower
	if linalg_ex.CrossProductSign(vertA.p, edge.vL.p, edge.vR.p) == 0 do return

	// ictResult - result sign is dependant on triangleA's orientation
	ict := linalg_ex.InCircleTest(vertA.p, edge.vL.p, edge.vR.p, vertB.p)
	if ict.i == 0 do return
	right_turn := linalg_ex.CrossProductSign(vertA.p, edge.vL.p, edge.vR.p) > 0
	if right_turn == (ict.i < 0) do return // if on or out of circle then exit

	// TRIANGLES HERE ARE **NOT** DELAUNAY COMPLIANT, SO MAKE THEM SO.

	// NOTE: ONCE WE BEGIN DELAUNAY COMPLIANCE, vL & vR WILL
	// NO LONGER REPRESENT LEFT AND RIGHT VERTEX ORIENTATION.
	// THIS IS MINOR PERFORMANCE EFFICIENCY IS SAFE AS LONG AS
	// THE TRIANGULATE() METHOD IS CALLED ONCE ONLY ON A GIVEN
	// SET OF PATHS

	// Flip edge: vL<->vertA, vR<->vertB
	edge.vL = vertA
	edge.vR = vertB

	edge.triA.edges[0] = edge
	for i in 1 ..< 3 {
		edge.triA.edges[i] = edgesA[i]
		ed := edgesA[i]
		if ed == nil do return .FORCELEGAL_EDGEA_NIL
		if IsLooseEdge(ed) do non_zero_append(&ctx.pendingDelaunayStack, ed) or_return
		// since each edge has its own triangleA and triangleB, we have to be careful
		// to update the correct one ...
		if ed.triA == edge.triA || ed.triB == edge.triA do continue

		if ed.triA == edge.triB do ed.triA = edge.triA
		else if ed.triB == edge.triB do ed.triB = edge.triA
		else do return .FORCELEGAL_UNKNOWN1
	}

	edge.triB.edges[0] = edge
	for i in 1 ..< 3 {
		edge.triB.edges[i] = edgesB[i]
		ed := edgesB[i]
		if ed == nil do return .FORCELEGAL_EDGEB_NIL
		if IsLooseEdge(ed) do non_zero_append(&ctx.pendingDelaunayStack, ed) or_return
		// since each edge has its own triangleA and triangleB, we have to be careful
		// to update the correct one ...
		if ed.triA == edge.triB || ed.triB == edge.triB do continue

		if ed.triA == edge.triA do ed.triA = edge.triB
		else if ed.triB == edge.triA do ed.triB = edge.triB
		else do return .FORCELEGAL_UNKNOWN2
	}
	return
}

@(private = "file")
CreateInnerLocMinLooseEdge :: proc(
	ctx: ^Context,
	vAbove: ^Vertex,
) -> (
	ed: ^Edge,
	err: Trianguate_Error,
) {
	if ctx.firstActive == nil do return nil, .FIRST_ACTIVE_MISSING

	xAbove := vAbove.p.x.i
	yAbove := vAbove.p.y.i

	e := ctx.firstActive
	eBelow: ^Edge = nil
	bestD: FixedDef = FixedDef {
		i = -1 << FixedDef.Fraction_Width,
	}

	for e != nil {
		in_range :=
			e.vL.p.x.i <= xAbove &&
			e.vR.p.x.i >= xAbove &&
			e.vB.p.y.i >= yAbove &&
			e.vB != vAbove &&
			e.vT != vAbove &&
			!(linalg_ex.CrossProductSign(e.vL.p, vAbove.p, e.vR.p) < 0)
		if in_range {
			d, d_ := linalg_ex.ShortestLength2Line(vAbove.p, e.vL.p, e.vR.p)
			if eBelow == nil ||
			   (d_.i > 0 ? d.i < fixed.mul(bestD, d_).i : d.i > fixed.mul(bestD, d_).i) {
				eBelow = e
				bestD = d
			}
		}
		e = e.nextE
	}

	if eBelow == nil do return nil, .EBELLOW_MISSING

	vBest: ^Vertex = eBelow.vT.p.y.i <= yAbove ? eBelow.vB : eBelow.vT
	xBest := vBest.p.x.i
	yBest := vBest.p.y.i

	// make sure no edges intersect 'vAbove' and 'vBest' ...
	// TODO: fActives is currently *unsorted* but consider making it
	// a tree structure based on each edge's left and right bounds
	e = ctx.firstActive
	if xBest < xAbove {
		for e != nil {
			if e.vR.p.x.i > xBest &&
			   e.vL.p.x.i < xAbove &&
			   e.vB.p.y.i > yAbove &&
			   e.vT.p.y.i < yBest &&
			   .intersect == linalg_ex.LinesIntersect3(e.vB.p, e.vT.p, vBest.p, vAbove.p, true) {
				vBest = e.vT.p.y.i > yAbove ? e.vT : e.vB
				xBest = vBest.p.x.i
				yBest = vBest.p.y.i
			}
			e = e.nextE
		}
	} else {
		for e != nil {
			if e.vR.p.x.i < xBest &&
			   e.vL.p.x.i > xAbove &&
			   e.vB.p.y.i > yAbove &&
			   e.vT.p.y.i < yBest &&
			   .intersect == linalg_ex.LinesIntersect3(e.vB.p, e.vT.p, vBest.p, vAbove.p, true) {
				vBest = e.vT.p.y.i > yAbove ? e.vT : e.vB
				xBest = vBest.p.x.i
				yBest = vBest.p.y.i
			}
			e = e.nextE
		}
	}
	ed = MakeEdge(ctx, vBest, vAbove, .loose) or_return
	return
}


@(private = "file")
MergeDupOrCollinearVertices :: proc(ctx: ^Context) -> (err: Trianguate_Error) {
	// note: this procedure may add new edges and change the
	// number of edges connected with a given vertex, but it
	// won't add or delete vertices (so it's safe to use iterators)
	// Precondition: all_verts must be sorted (y desc, then x asc)
	v1_idx := 0
	for i in 1 ..< len(ctx.all_verts) {
		if ctx.all_verts[i].p.x != ctx.all_verts[v1_idx].p.x ||
		   ctx.all_verts[i].p.y != ctx.all_verts[v1_idx].p.y {
			v1_idx = i
			continue
		}

		// merge v1 & v2
		vt := ctx.all_verts[v1_idx]
		v2 := ctx.all_verts[i]
		if !vt.innerLM || !v2.innerLM do vt.innerLM = false

		// in all of v2's edges, replace links to v2 with links to v1
		for e in v2.e {
			if e.vB == v2 {e.vB = vt} else {e.vT = vt}
			if e.vL == v2 {e.vL = vt} else {e.vR = vt}
		}
		// move all of v2's edges to v1
		for e in v2.e {
			non_zero_append(&vt.e, e) or_return
		}
		clear(&v2.e)

		// excluding horizontals, if pv.edges contains two edges
		// that are *collinear* and share the same bottom coords
		// but have different lengths, split the longer edge at
		// the top of the shorter edge ...
		for e, i in vt.e {
			e1 := e
			if IsHorizontal(e1) || e1.vB != vt do continue
			for j := i + 1; j < len(vt.e); j += 1 {
				e2 := vt.e[j]
				if e2.vB != vt || e1.vT.p.y.i == e2.vT.p.y.i || linalg_ex.CrossProductSign(e1.vT.p, vt.p, e2.vT.p) != 0 do continue
				// we have parallel edges, both heading up from vt.p.
				// split the longer edge at the top of the shorter edge.
				if e1.vT.p.y.i < e2.vT.p.y.i {
					SplitEdge(ctx, e1, e2) or_return
				} else {
					SplitEdge(ctx, e2, e1) or_return
				}
				break // because only two edges can be collinear
			}
		}
	}
	return
}


@(private = "file")
AddPath :: proc(ctx: ^Context, pts: [][2]FixedDef) -> (err: Trianguate_Error) {
	i0 := 0
	if !FindLocMinIdx(pts, &i0) do return

	iPrev := utils.Prev(i0, len(pts))
	for pts[iPrev].x == pts[i0].x && pts[iPrev].y == pts[i0].y {
		iPrev = utils.Prev(iPrev, len(pts))
	}

	iNext := utils.Next(i0, len(pts))
	i := i0
	for linalg_ex.CrossProductSign(pts[iPrev], pts[i], pts[iNext]) == 0 {
		FindLocMinIdx(pts, &i)
		if i == i0 do return
		iPrev = utils.Prev(i, len(pts))
		for pts[iPrev].x == pts[i].x && pts[iPrev].y == pts[i].y {
			iPrev = utils.Prev(iPrev, len(pts))
		}
		iNext = utils.Next(i, len(pts))
	}


	vert_cnt := len(ctx.all_verts)

	// we are now at the first legitimate locMin
	vert := MakeVertex(pts[i]) or_return

	non_zero_append(&ctx.all_verts, vert) or_return
	v0 := ctx.all_verts[vert_cnt]
	v0.innerLM = linalg_ex.CrossProductSign(pts[iPrev], pts[i], pts[iNext]) < 0
	vPrev := v0
	i = iNext

	for {
		non_zero_append(&ctx.loc_min_stack, vPrev) or_return // vPrev is a locMin here

		// update lowermostVertex ...
		if ctx.lowermostVertex == nil ||
		   vPrev.p.y.i > ctx.lowermostVertex.p.y.i ||
		   (vPrev.p.y.i == ctx.lowermostVertex.p.y.i && vPrev.p.x.i < ctx.lowermostVertex.p.x.i) {
			ctx.lowermostVertex = vPrev
		}

		iNext = utils.Next(i, len(pts))
		//https://github.com/AngusJohnson/Clipper2/pull/1077
		for linalg_ex.CrossProductSign(vPrev.p, pts[i], pts[iNext]) == 0 { 	//skips collinear path (vPrev, pts[i], pts[iNext] is straight)
			i = iNext
			if i == i0 do break
			iNext = utils.Next(i, len(pts))
		}
		if i == i0 do break
		// if linalg_ex.CrossProductSign(vPrev.p, pts[i], pts[iNext]) == 0 {
		// 	i = iNext
		// 	continue
		// }

		for pts[i].y.i <= vPrev.p.y.i { 	// ascend up next bound to LocMax
			vert := MakeVertex(pts[i]) or_return

			non_zero_append(&ctx.all_verts, vert) or_return
			v := ctx.all_verts[len(ctx.all_verts) - 1]

			MakeEdge(ctx, vPrev, v, .ascend)
			vPrev = v
			i = iNext

			for iNext = utils.Next(i, len(pts));
			    linalg_ex.CrossProductSign(vPrev.p, pts[i], pts[iNext]) == 0;
			    iNext = utils.Next(i, len(pts)) {
				i = iNext
			}
		}


		// Now at a locMax, so descend to next locMin
		vPrevPrev := vPrev
		for i != i0 && pts[i].y.i >= vPrev.p.y.i {
			vert := MakeVertex(pts[i]) or_return
			non_zero_append(&ctx.all_verts, vert) or_return
			v := ctx.all_verts[len(ctx.all_verts) - 1]
			MakeEdge(ctx, v, vPrev, .descend)

			vPrevPrev = vPrev
			vPrev = v
			i = iNext

			for iNext = utils.Next(i, len(pts));
			    linalg_ex.CrossProductSign(vPrev.p, pts[i], pts[iNext]) == 0;
			    iNext = utils.Next(i, len(pts)) {
				i = iNext
			}
		}

		if i == i0 do break
		// Determine inner local minima per-locMin (Clipper2 behavior).
		// Only set to true when the vertex turns left at the next locMin.
		if linalg_ex.CrossProductSign(vPrevPrev.p, vPrev.p, pts[i]) < 0 {
			vPrev.innerLM = true
		}
	}
	MakeEdge(ctx, v0, vPrev, .descend) or_return

	len_fin := len(ctx.all_verts) - vert_cnt // finally, ignore this path if is not a polygon or too small
	i = vert_cnt
	if len_fin < 3 { 	//Skipped the original polygon-size check for now — needs testing.
		for j := vert_cnt; j < len(ctx.all_verts); j += 1 {
			clear(&ctx.all_verts[j].e) // flag to ignore //TODO 갯수만 0개로 하면 되는지 다음에 확인해보자
		}
	}

	return
}

@(private = "file")
AddPaths :: proc(ctx: ^Context) -> (err: Trianguate_Error) {
	total_vert_count := 0
	for p in ctx.polys {
		total_vert_count += len(p)
	}
	if total_vert_count == 0 do return .NO_PATHS
	non_zero_reserve(&ctx.all_verts, cap(ctx.all_verts) + total_vert_count)
	non_zero_reserve(&ctx.all_edges, cap(ctx.all_edges) + total_vert_count)

	for p in ctx.polys {
		err := AddPath(ctx, p)
		if err != nil && err != .NO_PATHS do return err
	}
	if len(ctx.all_verts) <= 2 do return .NO_PATHS
	return
}

@(private = "file")
FindLocMinIdx :: proc "contextless" (pts: [][2]FixedDef, in_out_idx: ^int) -> bool {
	if len(pts) < 3 do return false

	i0 := in_out_idx^
	idx := in_out_idx
	next := utils.Next(idx^, len(pts))
	for ; pts[next].y.i <= pts[idx^].y.i; next = utils.Next(next, len(pts)) {
		idx^ = next
		if idx^ == i0 do return false // fails if path is completely horizontal
	}

	for pts[next].y.i >= pts[idx^].y.i {
		idx^ = next
		next = utils.Next(next, len(pts))
	}

	return true
}

@(private = "file")
CreateTriangle :: proc(
	ctx: ^Context,
	e1, e2, e3: ^Edge,
) -> (
	res: ^Triangle,
	err: Trianguate_Error,
) {
	res = new(Triangle, context.temp_allocator) or_return
	res.edges = {e1, e2, e3}
	non_zero_append(&ctx.all_tris, res) or_return
	// nb: only expire loose edges when both sides of these edges have triangles.
	for i in 0 ..< 3 {
		ed := res.edges[i]
		if ed.triA != nil {
			ed.triB = res
			RemoveEdgeFromActives(ctx, ed)
		} else {
			ed.triA = res
			// this is the edge's first triangle, so only remove
			// this edge from actives if it's a fixed edge.
			if ed.kind != .loose do RemoveEdgeFromActives(ctx, ed)
		}
	}
	return
}

@(private = "file")
EdgeContains :: proc "contextless" (edge: ^Edge, v: ^Vertex) -> EdgeContainsResult {
	if edge.vL == v do return .left
	if edge.vR == v do return .right
	return .neither
}

@(private = "file")
RemoveEdgeFromVertex :: proc(vert: ^Vertex, edge: ^Edge) -> (err: Trianguate_Error) {
	for e, i in vert.e {
		if e == edge {
			ordered_remove(&vert.e, i)
			return nil
		}
	}
	return .NO_EDGE_IN_VERTEX
}

@(private = "file")
IsHorizontal :: proc "contextless" (e: ^Edge) -> bool {
	return e.vB.p.y.i == e.vT.p.y.i
}

@(private = "file")
MakeEdge :: proc(
	ctx: ^Context,
	v1, v2: ^Vertex,
	kind: EdgeKind,
) -> (
	res: ^Edge,
	err: Trianguate_Error,
) {
	if len(ctx.all_edges) >= int(max(u32)) do return nil, .TOO_MANY_EDGES
	non_zero_append(&ctx.all_edges, new(Edge, context.temp_allocator) or_return) or_return

	ed := ctx.all_edges[len(ctx.all_edges) - 1]
	if v1.p.y.i >= v2.p.y.i {
		ed.vB = v1; ed.vT = v2
	} else {
		ed.vB = v2; ed.vT = v1
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

@(private = "file")
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

@(private = "file")
RemoveEdgeFromActives :: proc(ctx: ^Context, edge: ^Edge) -> (err: Trianguate_Error) {
	RemoveEdgeFromVertex(edge.vB, edge) or_return
	RemoveEdgeFromVertex(edge.vT, edge) or_return

	prev := edge.prevE
	next := edge.nextE

	if next != nil do next.prevE = prev
	if prev != nil do prev.nextE = next

	edge.isActive = false
	if ctx.firstActive == edge do ctx.firstActive = next
	return nil
}

@(private = "file")
SplitEdge :: proc(ctx: ^Context, longE, shortE: ^Edge) -> (err: Trianguate_Error) {
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

@(private = "file")
RemoveIntersection :: proc(ctx: ^Context, e1: ^Edge, e2: ^Edge) -> (err: Trianguate_Error) {
	v: ^Vertex = e1.vL
	tmpE: ^Edge = e2

	d, d_ := linalg_ex.ShortestLength2Line(e1.vL.p, e2.vL.p, e2.vR.p)
	d2, d2_ := linalg_ex.ShortestLength2Line(e1.vR.p, e2.vL.p, e2.vR.p)
	if fixed.div(d2, d2_).i < fixed.div(d, d_).i {
		d = d2; d_ = d2_
		v = e1.vR
	}

	d2, d2_ = linalg_ex.ShortestLength2Line(e2.vL.p, e1.vL.p, e1.vR.p)
	if fixed.div(d2, d2_).i < fixed.div(d, d_).i {
		d = d2; d_ = d2_
		tmpE = e1
		v = e2.vL
	}

	d2, d2_ = linalg_ex.ShortestLength2Line(e2.vR.p, e1.vL.p, e1.vR.p)
	if fixed.div(d2, d2_).i < fixed.div(d, d_).i {
		d = d2; d_ = d2_
		tmpE = e1
		v = e2.vR
	}
	if d.i > 1 << FixedDef.Fraction_Width do return .PATHS_INTERSECTS

	v2 := tmpE.vT
	RemoveEdgeFromVertex(v2, tmpE) or_return
	if tmpE.vL == v2 do tmpE.vL = v
	else do tmpE.vR = v
	tmpE.vT = v
	non_zero_append(&v.e, tmpE) or_return
	v.innerLM = false

	if tmpE.vB.innerLM && GetLocMinAngleCheck(tmpE.vB) do tmpE.vB.innerLM = false
	MakeEdge(ctx, v, v2, tmpE.kind) or_return
	return nil
}

@(private = "file")
GetLocMinAngleCheck :: proc(v: ^Vertex) -> bool {
	// Original (atan2): return GetAngle(des.vT.p, v.p, asc.vT.p) <= 0.
	asc, des: int
	if v.e[0].kind == .ascend {asc = 0; des = 1} else {des = 0; asc = 1}
	// Check whether the local-minimum angle is <= 0: sign of 2D cross product cp = (b - a) x (b - c).
	a := v.e[des].vT.p
	b := v.p
	c := v.e[asc].vT.p

	abx := fixed.sub(b.x, a.x)
	aby := fixed.sub(b.y, a.y)
	bcx := fixed.sub(b.x, c.x)
	bcy := fixed.sub(b.y, c.y)

	cp := fixed.sub(fixed.mul(abx, bcy), fixed.mul(aby, bcx))
	return cp.i <= 0
}

@(private = "file")
vertex_index :: proc "contextless" (poly: [][][2]FixedDef, v: ^Vertex) -> u32 {
	all_idx := 0
	for i in 0 ..< len(poly) {
		for j in 0 ..< len(poly[i]) {
			if poly[i][j].x == v.p.x && poly[i][j].y == v.p.y do return u32(all_idx + j)
		}
		all_idx += len(poly[i])
	}
	return 0
}

@(private = "file")
EdgeCompleted :: proc "contextless" (e: ^Edge) -> bool {
	if e.triA == nil do return false
	if e.triB != nil do return true
	return e.kind != .loose
}

@(private = "file")
IsLooseEdge :: proc "contextless" (e: ^Edge) -> bool {
	return e.kind == .loose
}

@(private = "file")
IsLeftEdge :: proc "contextless" (e: ^Edge) -> bool {
	return e.kind == .ascend
}

@(private = "file")
IsRightEdge :: proc "contextless" (e: ^Edge) -> bool {
	return e.kind == .descend
}

@(private = "file")
FindLinkingEdge :: proc "contextless" (vert1, vert2: ^Vertex, prefer_ascending: bool) -> ^Edge {
	res: ^Edge = nil
	for e in vert1.e {
		if e.vL == vert2 || e.vR == vert2 {
			if e.kind == .loose || (e.kind == .ascend) == prefer_ascending do return e
			res = e
		}
	}
	return res
}

@(private = "file")
MakeVertex :: proc(p: [2]FixedDef) -> (res: ^Vertex, err: Trianguate_Error) {
	res = new_clone(Vertex{p = p, innerLM = false}, context.temp_allocator) or_return
	res.e = make([dynamic]^Edge, context.temp_allocator) or_return
	non_zero_reserve(&res.e, 2) or_return
	return
}

TrianguatePolygons_Fixed :: proc(
	poly: [][][2]FixedDef,
	allocator := context.allocator,
) -> (
	indices: []u32,
	err: Trianguate_Error,
) {
	return TrianguatePolygons_Fixed_Impl(poly, allocator)
}

TrianguatePolygons :: proc(
	poly: [][][2]$T,
	allocator := context.allocator,
) -> (
	indices: []u32,
	err: Trianguate_Error,
) where intrinsics.type_is_float(T) {
	return TrianguatePolygons_Impl(poly, allocator)
}

