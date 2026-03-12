package geometry

import "base:intrinsics"
import "base:runtime"
import "core:math"
import "core:math/linalg"
import "core:mem"
import "core:slice"

import utils "shared:utils_private"
import "shared:utils_private/fixed_bcd"

__Trianguate_Error :: enum {
	FIRST_ACTIVE_MISSING,
	EBELLOW_MISSING,
	PATHS_INTERSECTS,
	FORCELEGAL_EDGEA_NIL,
	FORCELEGAL_EDGEB_NIL,
	FORCELEGAL_UNKNOWN1,
	FORCELEGAL_UNKNOWN2,
}

Trianguate_Error :: union #shared_nil {
	__Geometry_Error,
	__Trianguate_Error,
	runtime.Allocator_Error,
}

// Precondition: ctx.all_edges must be sorted ascending on edge.vL.p.x
FixupEdgeIntersects :: proc(ctx: ^Context($T)) -> (err: Trianguate_Error) {
	for i1 in 0 ..< len(ctx.all_edges) {
		e1 := ctx.all_edges[i1]
		for i2 in i1 + 1 ..< len(ctx.all_edges) {
			e2 := ctx.all_edges[i2]
			if e2.vL.p.x.i >= e1.vR.p.x.i {
				break
			}
			if e2.vT.p.y.i < e1.vB.p.y.i && e2.vB.p.y.i > e1.vT.p.y.i {
				if LinesIntersect3(e2.vL.p, e2.vR.p, e1.vL.p, e1.vR.p) == .intersect {
					ok, gerr := RemoveIntersection(ctx, e2, e1)
					if gerr != nil do return Geometry_To_Triangulate_Error(gerr)
					if !ok do return __Trianguate_Error.PATHS_INTERSECTS
				}
			}
		}
	}
	return nil
}

TrianguatePolygons_Fixed :: proc(
	poly: [][][2]$T,
	allocator := context.allocator,
) -> (
	indices: []u32,
	err: Trianguate_Error,
) where intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	ctx := Context(T) {
		all_verts            = make([dynamic]^Vertex(T), context.temp_allocator) or_return,
		all_tris             = make([dynamic]^Triangle(T), context.temp_allocator) or_return,
		all_edges            = make([dynamic]^Edge(T), context.temp_allocator) or_return,
		pendingDelaunayStack = make([dynamic]^Edge(T), context.temp_allocator) or_return,
		loc_min_stack        = make([dynamic]^Vertex(T), context.temp_allocator) or_return,
		horz_edge_stack      = make([dynamic]^Edge(T), context.temp_allocator) or_return,
		polys                = poly,
	}
	non_zero_reserve(&ctx.all_edges, len(poly)) or_return

	AddPaths(&ctx) or_return

	if ctx.lowermostVertex != nil && ctx.lowermostVertex.innerLM {
		lm: ^Vertex(T)
		for len(ctx.loc_min_stack) > 0 {
			lm = ctx.loc_min_stack[len(ctx.loc_min_stack) - 1]
			lm.innerLM = !lm.innerLM
			non_zero_resize(&ctx.loc_min_stack, len(ctx.loc_min_stack) - 1)
		}
		for &e in ctx.all_edges {
			if e.kind == .ascend do e.kind = .descend
			else do e.kind = .ascend
		}
	} else {
		clear(&ctx.loc_min_stack)
	}

	slice.sort_by(ctx.all_edges[:], proc(a, b: ^Edge(T)) -> bool {
		return a.vL.p.x.i < b.vL.p.x.i
	})
	FixupEdgeIntersects(&ctx) or_return
	slice.sort_by(ctx.all_verts[:], proc(a, b: ^Vertex(T)) -> bool {
		if a.p.y.i == b.p.y.i do return a.p.x.i < b.p.x.i
		return a.p.y.i > b.p.y.i
	})
	MergeDupOrCollinearVertices(&ctx) or_return

	currY: T = ctx.all_verts[0].p.y
	for &v in ctx.all_verts {
		if len(v.e) == 0 do continue
		if v.p.y.i != currY.i {
			// JOIN AN INNER LOCMIN WITH A SUITABLE EDGE BELOW
			for len(ctx.loc_min_stack) > 0 {
				lm := ctx.loc_min_stack[len(ctx.loc_min_stack) - 1]
				non_zero_resize(&ctx.loc_min_stack, len(ctx.loc_min_stack) - 1)
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
		non_zero_resize(&ctx.horz_edge_stack, len(ctx.horz_edge_stack) - 1)

		if !EdgeCompleted(e) && e.vB == e.vL {
			DoTriangulateLeft(&ctx, e, e.vB, currY) or_return
		}
	}

	// Delaunay 적합 삼각형으로 변환
	for len(ctx.pendingDelaunayStack) > 0 {
		e := ctx.pendingDelaunayStack[len(ctx.pendingDelaunayStack) - 1]
		non_zero_resize(&ctx.pendingDelaunayStack, len(ctx.pendingDelaunayStack) - 1)
		ForceLegal(&ctx, e) or_return
	}


	// Convert triangles to index buffer; skip degenerate, ensure CW winding
	idx_buf := make([dynamic]u32, context.temp_allocator) or_return
	defer delete(idx_buf)
	for tri in ctx.all_tris {
		vs := triangle_vertices(tri)
		p0, p1, p2 := vs[0].p, vs[1].p, vs[2].p
		cps := CrossProductSign(p0, p1, p2)
		if cps == 0 do continue // skip degenerate triangles
		i0 := vertex_index(&ctx, vs[0])
		i1 := vertex_index(&ctx, vs[1])
		i2 := vertex_index(&ctx, vs[2])
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
triangle_vertices :: proc "contextless" (tri: ^Triangle($T)) -> [3]^Vertex(T) {
	e0, e1 := tri.edges[0], tri.edges[1]
	vs: [3]^Vertex(T)
	vs[0] = e0.vL
	vs[1] = e0.vR
	if e1.vL == vs[0] || e1.vL == vs[1] do vs[2] = e1.vR
	else do vs[2] = e1.vL
	return vs
}

@(private = "file")
DoTriangulateLeft :: proc(
	ctx: ^Context($T),
	edge: ^Edge(T),
	pivot: ^Vertex(T),
	minY: T,
) -> (
	err: Trianguate_Error,
) where intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	vAlt: ^Vertex(T) = nil
	eAlt: ^Edge(T) = nil
	v := edge.vT if edge.vB == pivot else edge.vB
	for e in pivot.e {
		if e == edge || !e.isActive do continue
		vX := e.vB if e.vT == pivot else e.vT
		if vX == v do continue
		cps := CrossProductSign(v.p, pivot.p, vX.p)
		if cps == 0 {
			// if pivot is between v and vX then continue
			if (v.p.x.i > pivot.p.x.i) == (pivot.p.x.i > vX.p.x.i) do continue
		} else if cps > 0 || (vAlt != nil && !(CrossProductSign(vX.p, pivot.p, vAlt.p) < 0)) {
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
		gerr: Geometry_Error
		eX, gerr = MakeEdge(ctx, vAlt, v, .loose)
		if gerr != nil do return Geometry_To_Triangulate_Error(gerr)
	}
	CreateTriangle(ctx, edge, eAlt, eX) or_return
	if !EdgeCompleted(eX) do DoTriangulateLeft(ctx, eX, vAlt, minY) or_return
	return
}

@(private = "file")
DoTriangulateRight :: proc(
	ctx: ^Context($T),
	edge: ^Edge(T),
	pivot: ^Vertex(T),
	minY: T,
) -> (
	err: Trianguate_Error,
) where intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	vAlt: ^Vertex(T) = nil
	eAlt: ^Edge(T) = nil
	v := edge.vT if edge.vB == pivot else edge.vB
	for e in pivot.e {
		if e == edge || !e.isActive do continue
		vX := e.vB if e.vT == pivot else e.vT
		if vX == v do continue
		cps := CrossProductSign(v.p, pivot.p, vX.p)
		if cps == 0 {
			// if pivot is between v and vX then continue;
			// nb: this is important for both horiz and non-horiz collinear
			if (v.p.x.i > pivot.p.x.i) == (pivot.p.x.i > vX.p.x.i) do continue
		} else if cps < 0 || (vAlt != nil && !(CrossProductSign(vX.p, pivot.p, vAlt.p) > 0)) { 	// else if right-turning or not the best edge, then continue
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
		gerr: Geometry_Error
		eX, gerr = MakeEdge(ctx, vAlt, v, .loose)
		if gerr != nil do return Geometry_To_Triangulate_Error(gerr)
	}

	CreateTriangle(ctx, edge, eX, eAlt) or_return
	if !EdgeCompleted(eX) do DoTriangulateRight(ctx, eX, vAlt, minY) or_return

	return
}

@(private = "file")
ForceLegal :: proc(ctx: ^Context($T), edge: ^Edge(T)) -> (err: Trianguate_Error) {
	if edge.triA == nil || edge.triB == nil do return
	vertA: ^Vertex(T) = nil
	vertB: ^Vertex(T) = nil
	edgesA: [3]^Edge(T) = {nil, nil, nil}
	edgesB: [3]^Edge(T) = {nil, nil, nil}

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
	if CrossProductSign(vertA.p, edge.vL.p, edge.vR.p) == 0 do return

	// ictResult - result sign is dependant on triangleA's orientation
	ict := InCircleTest(vertA.p, edge.vL.p, edge.vR.p, vertB.p)
	if ict.i == 0 do return
	right_turn := CrossProductSign(vertA.p, edge.vL.p, edge.vR.p) > 0
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
	ctx: ^Context($T),
	vAbove: ^Vertex(T),
) -> (
	ed: ^Edge(T),
	err: Trianguate_Error,
) where intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	if ctx.firstActive == nil do return nil, .FIRST_ACTIVE_MISSING
	xAbove := vAbove.p.x.i
	yAbove := vAbove.p.y.i
	e := ctx.firstActive
	eBelow: ^Edge(T) = nil
	bestD: T = {}
	for e != nil {
		in_range :=
			e.vL.p.x.i <= xAbove &&
			e.vR.p.x.i >= xAbove &&
			e.vB.p.y.i >= yAbove &&
			e.vB != vAbove &&
			e.vT != vAbove &&
			!(CrossProductSign(e.vL.p, vAbove.p, e.vR.p) < 0)
		if in_range {
			d, d_ := ShortestLength2Line(vAbove.p, e.vL.p, e.vR.p)
			if eBelow == nil ||
			   (d_.i > 0 ? d.i < fixed_bcd.mul(bestD, d_).i : d.i > fixed_bcd.mul(bestD, d_).i) {
				eBelow = e
				bestD = d
			}
		}
		e = e.nextE
	}
	if eBelow == nil do return nil, .EBELLOW_MISSING
	vBest: ^Vertex(T) = eBelow.vB if eBelow.vT.p.y.i <= yAbove else eBelow.vT
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
			   e.vT.p.y.i < yBest {
				if res := LinesIntersect3(e.vB.p, e.vT.p, vBest.p, vAbove.p); res == .intersect {
					vBest = e.vT if e.vT.p.y.i > yAbove else e.vB
					xBest = vBest.p.x.i
					yBest = vBest.p.y.i
				}
			}
			e = e.nextE
		}
	} else {
		for e != nil {
			if e.vR.p.x.i < xBest &&
			   e.vL.p.x.i > xAbove &&
			   e.vB.p.y.i > yAbove &&
			   e.vT.p.y.i < yBest {
				if res := LinesIntersect3(e.vB.p, e.vT.p, vBest.p, vAbove.p); res == .intersect {
					vBest = e.vT if e.vT.p.y.i > yAbove else e.vB
					xBest = vBest.p.x.i
					yBest = vBest.p.y.i
				}
			}
			e = e.nextE
		}
	}
	gerr: Geometry_Error
	ed, gerr = MakeEdge(ctx, vBest, vAbove, .loose)
	if gerr != nil do return ed, Geometry_To_Triangulate_Error(gerr)
	return
}


@(private = "file")
MergeDupOrCollinearVertices :: proc(ctx: ^Context($T)) -> (err: Trianguate_Error) {
	// note: this procedure may add new edges and change the
	// number of edges connected with a given vertex, but it
	// won't add or delete vertices (so it's safe to use iterators)
	// Precondition: all_verts must be sorted (y desc, then x asc)
	v1_idx := 0
	for i in 1 ..< len(ctx.all_verts) {
		if !fixed_bcd.eq(ctx.all_verts[i].p.x, ctx.all_verts[v1_idx].p.x) ||
		   !fixed_bcd.eq(ctx.all_verts[i].p.y, ctx.all_verts[v1_idx].p.y) {
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
				if e2.vB != vt || e1.vT.p.y.i == e2.vT.p.y.i || CrossProductSign(e1.vT.p, vt.p, e2.vT.p) != 0 do continue
				// we have parallel edges, both heading up from vt.p.
				// split the longer edge at the top of the shorter edge.
				if e1.vT.p.y.i < e2.vT.p.y.i {
					gerr := SplitEdge(ctx, e1, e2)
					if gerr != nil do return Geometry_To_Triangulate_Error(gerr)
				} else {
					gerr := SplitEdge(ctx, e2, e1)
					if gerr != nil do return Geometry_To_Triangulate_Error(gerr)
				}
				break // because only two edges can be collinear
			}
		}
	}
	return
}


@(private = "file")
AddPath :: proc(
	ctx: ^Context($T),
	pts: [][2]T,
) -> (
	err: Trianguate_Error,
) where intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	i0 := 0
	if !FindLocMinIdx(pts, &i0) do return
	iPrev := utils.Prev(i0, len(pts))
	for pts[iPrev].x == pts[i0].x && pts[iPrev].y == pts[i0].y {
		iPrev = utils.Prev(iPrev, len(pts))
	}
	iNext := utils.Next(i0, len(pts))
	i := i0
	for CrossProductSign(pts[iPrev], pts[i], pts[iNext]) == 0 {
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
	vert, vert_err := MakeVertex(pts[i])
	if vert_err != nil do return Geometry_To_Triangulate_Error(vert_err)
	non_zero_append(&ctx.all_verts, vert) or_return
	v0 := ctx.all_verts[vert_cnt]
	v0.innerLM = CrossProductSign(pts[iPrev], pts[i], pts[iNext]) < 0
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
		if CrossProductSign(vPrev.p, pts[i], pts[iNext]) == 0 { 	//skips collinear path (vPrev, pts[i], pts[iNext] is straight)
			i = iNext
			continue
		}


		for pts[i].y.i <= vPrev.p.y.i { 	// ascend up next bound to LocMax
			vert, vert_err := MakeVertex(pts[i])
			if vert_err != nil do return Geometry_To_Triangulate_Error(vert_err)
			non_zero_append(&ctx.all_verts, vert) or_return
			v := ctx.all_verts[len(ctx.all_verts) - 1]

			MakeEdge(ctx, vPrev, v, .ascend)
			vPrev = v
			i = iNext

			for iNext = utils.Next(i, len(pts));
			    CrossProductSign(vPrev.p, pts[i], pts[iNext]) == 0; {
				i = iNext
				iNext = utils.Next(i, len(pts))
			}
		}


		// Now at a locMax, so descend to next locMin
		vPrevPrev := vPrev
		for i != i0 && pts[i].y.i >= vPrev.p.y.i {
			vert, vert_err := MakeVertex(pts[i])
			if vert_err != nil do return Geometry_To_Triangulate_Error(vert_err)
			non_zero_append(&ctx.all_verts, vert) or_return
			v := ctx.all_verts[len(ctx.all_verts) - 1]
			MakeEdge(ctx, v, vPrev, .descend)
			vPrevPrev = vPrev
			vPrev = v
			i = iNext

			for iNext = utils.Next(i, len(pts));
			    CrossProductSign(vPrev.p, pts[i], pts[iNext]) == 0; {
				i = iNext
				iNext = utils.Next(i, len(pts))
			}
		}

		if i == i0 do break
		// Determine inner local minima per-locMin (Clipper2 behavior).
		// Only set to true when the vertex turns left at the next locMin.
		if CrossProductSign(vPrevPrev.p, vPrev.p, pts[i]) < 0 {
			vPrev.innerLM = true
		}
	}
	_, ed_err := MakeEdge(ctx, v0, vPrev, .descend)
	if ed_err != nil do return Geometry_To_Triangulate_Error(ed_err)

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
Geometry_To_Triangulate_Error :: proc "contextless" (err: Geometry_Error) -> Trianguate_Error {
	switch g in err {
	case __Geometry_Error:
		return err.(__Geometry_Error)
	case runtime.Allocator_Error:
		return err.(runtime.Allocator_Error)
	}
	return nil
}

@(private = "file")
AddPaths :: proc(ctx: ^Context($T)) -> (err: Trianguate_Error) {
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
FindLocMinIdx :: proc "contextless" (
	pts: [][2]$T,
	in_out_idx: ^int,
) -> bool where intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
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
CreateTriangle :: proc(ctx: ^Context($T), e1, e2, e3: ^Edge(T)) -> (err: Trianguate_Error) {
	tri := new(Triangle(T), context.temp_allocator) or_return
	tri.edges = {e1, e2, e3}
	non_zero_append(&ctx.all_tris, tri) or_return
	// nb: only expire loose edges when both sides of these edges have triangles.
	for i in 0 ..< 3 {
		ed := tri.edges[i]
		if ed.triA != nil {
			ed.triB = tri
			// Second triangle: remove from actives only for loose (fixed already removed at triA).
			// Clipper2 calls RemoveEdgeFromActives here unconditionally -> double-removal for fixed edges.
			if ed.kind == .loose do RemoveEdgeFromActives(ctx, ed)
		} else {
			ed.triA = tri
			// this is the edge's first triangle, so only remove
			// this edge from actives if it's a fixed edge.
			if ed.kind != .loose do RemoveEdgeFromActives(ctx, ed)
		}
	}
	return
}

