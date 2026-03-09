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

__Trianguate_Error :: enum {
	FIRST_ACTIVE_MISSING,
	EBELLOW_MISSING,
}

Trianguate_Error :: union #shared_nil {
	__Geometry_Error,
	__Trianguate_Error,
	runtime.Allocator_Error,
}


TrianguatePolygons_Fixed :: proc(
	poly: [][][2]FixedDef,
	polyCCW: []PolyOrientation = nil,
	allocator := context.allocator,
) -> (
	indices: []u32,
	err: Trianguate_Error,
) {
	ctx := Context {
		all_verts            = make([dynamic]Vertex, context.temp_allocator) or_return,
		all_tris             = make([dynamic]^Triangle, context.temp_allocator) or_return,
		all_edges            = make([dynamic]^Edge, context.temp_allocator) or_return,
		pendingDelaunayStack = make([dynamic]^Edge, context.temp_allocator) or_return,
		loc_min_stack        = make([dynamic]^Vertex, context.temp_allocator) or_return,
		horz_edge_stack      = make([dynamic]^Edge, context.temp_allocator) or_return,
		polys                = poly,
	}

	ctx.polyCCW = polyCCW
	if ctx.polyCCW == nil { 	//polyCCW 정보 (폴리곤이 구멍인지 아닌지 여부) 가 없으면 직접 만들어야 한다.
		ctx.polyCCW = utils.make_non_zeroed_slice(
			[]PolyOrientation,
			len(poly),
			context.temp_allocator,
		) or_return
		for &p, i in ctx.polyCCW {
			p = GetPolygonOrientation(poly[i])
		}
	} else {
		assert(len(poly) == len(ctx.polyCCW))
	}
	non_zero_reserve(&ctx.all_edges, len(poly)) or_return

	AddPaths(&ctx) or_return


	// if necessary fix path orientation because the algorithm
	// expects clockwise outer paths and counter-clockwise inner paths
	if ctx.lowermostVertex.innerLM {
		// the orientation of added paths must be wrong, so
		// 1. reverse innerLM flags ...
		lm: ^Vertex
		for len(ctx.loc_min_stack) > 0 {
			lm = ctx.loc_min_stack[len(ctx.loc_min_stack) - 1]
			lm.innerLM = !lm.innerLM
			non_zero_resize(&ctx.loc_min_stack, len(ctx.loc_min_stack) - 1)
		}
		// 2. swap edge kinds
		for &e in ctx.all_edges {
			if e.kind == .ascend {
				e.kind = .descend
			} else {
				e.kind = .ascend
			}
		}
	} else {
		// path orientation is fine so ...
		clear(&ctx.loc_min_stack)
	}

	slice.sort_by(ctx.all_edges[:], proc(a, b: ^Edge) -> bool {
		return a.vL.p.x.i < b.vL.p.x.i
	})
	slice.sort_by(ctx.all_verts[:], proc(a, b: Vertex) -> bool {
		if a.p.y.i == b.p.y.i do return a.p.x.i < b.p.x.i
		return a.p.y.i > b.p.y.i
	})
	MergeDupOrCollinearVertices(&ctx) or_return


	currY := ctx.all_verts[0].p.y.i

	for &v in ctx.all_verts {
		if len(v.e) == 0 do continue
		if v.p.y.i != currY {
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
			currY = v.p.y.i
		}

		// inner edge loop (내림차순 인덱스 — 루프 중 edges 추가/삭제에 안전)
		for i := len(v.e) - 1; i >= 0; i -= 1 {
			if i >= len(v.e) do continue // v.e 는 루프 중에 변경될 수 있음
			e := v.e[i]
			if EdgeCompleted(e) || IsLooseEdge(e) do continue

			if &v == e.vB {
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
					DoTriangulateLeft(&ctx, e, e.vB, v.p.y.i) or_return
				} else {
					DoTriangulateRight(&ctx, e, e.vB, v.p.y.i) or_return
				}
			}
		} // inner edge loop

		if v.innerLM {
			non_zero_append(&ctx.loc_min_stack, &v) or_return
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

// Matches Clipper2 PathFromTriangle: uses vL/vR of edges
@(private = "file")
triangle_vertices :: proc "contextless" (tri: ^Triangle) -> [3]^Vertex {
	e0, e1 := tri.edges[0], tri.edges[1]
	vs: [3]^Vertex
	vs[0] = e0.vL
	vs[1] = e0.vR
	if e1.vL == vs[0] || e1.vL == vs[1] {
		vs[2] = e1.vR
	} else {
		vs[2] = e1.vL
	}
	return vs
}

@(private = "file")
DoTriangulateLeft :: proc(
	ctx: ^Context,
	edge: ^Edge,
	pivot: ^Vertex,
	minY: i64,
) -> (
	err: Trianguate_Error,
) {
	vAlt: ^Vertex = nil
	eAlt: ^Edge = nil
	v := edge.vT if edge.vB == pivot else edge.vB
	for e in pivot.e {
		if e == edge || !e.isActive do continue
		vX := e.vB if e.vT == pivot else e.vT
		if vX == v do continue
		cps := CrossProductSign(v.p, pivot.p, vX.p)
		if cps == 0 {
			// if pivot is between v and vX then continue
			if (v.p.x.i > pivot.p.x.i) == (pivot.p.x.i > vX.p.x.i) do continue
		} else if cps > 0 || (vAlt != nil && CrossProductSign(vX.p, pivot.p, vAlt.p) >= 0) {
			continue
		}
		vAlt = vX
		eAlt = e
	}
	if vAlt == nil || vAlt.p.y.i < minY do return

	if vAlt.p.y.i < pivot.p.y.i {
		if IsLeftEdge(eAlt) do return
	} else if vAlt.p.y.i > pivot.p.y.i {
		if IsRightEdge(eAlt) do return
	}
	eX := FindLinkingEdge(vAlt, v, vAlt.p.y.i < v.p.y.i)
	if eX == nil {
		if vAlt.p.y.i == v.p.y.i && v.p.y.i == minY && HorizontalBetween(ctx, vAlt, v) != nil {
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
	ctx: ^Context,
	edge: ^Edge,
	pivot: ^Vertex,
	minY: i64,
) -> (
	err: Trianguate_Error,
) {
	vAlt: ^Vertex = nil
	eAlt: ^Edge = nil
	v := edge.vT if edge.vB == pivot else edge.vB
	for e in pivot.e {
		if e == edge || !e.isActive do continue
		vX := e.vB if e.vT == pivot else e.vT
		if vX == v do continue
		cps := CrossProductSign(v.p, pivot.p, vX.p)
		if cps == 0 {
			if (v.p.x.i > pivot.p.x.i) == (pivot.p.x.i > vX.p.x.i) do continue
		} else if cps < 0 || (vAlt != nil && CrossProductSign(vX.p, pivot.p, vAlt.p) <= 0) {
			continue
		}
		vAlt = vX
		eAlt = e
	}
	if vAlt == nil || vAlt.p.y.i < minY do return

	if vAlt.p.y.i < pivot.p.y.i {
		if IsRightEdge(eAlt) do return
	} else if vAlt.p.y.i > pivot.p.y.i {
		if IsLeftEdge(eAlt) do return
	}
	eX := FindLinkingEdge(vAlt, v, vAlt.p.y.i > v.p.y.i)
	if eX == nil {
		if vAlt.p.y.i == v.p.y.i && v.p.y.i == minY && HorizontalBetween(ctx, vAlt, v) != nil {
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

	if CrossProductSign(vertA.p, edge.vL.p, edge.vR.p) == 0 do return
	ict := InCircleTest(vertA.p, edge.vL.p, edge.vR.p, vertB.p)
	if ict.i == 0 do return
	right_turn := CrossProductSign(vertA.p, edge.vL.p, edge.vR.p) > 0
	if right_turn == (ict.i < 0) do return


	// Flip edge: vL<->vertA, vR<->vertB
	edge.vL = vertA
	edge.vR = vertB
	edge.triA.edges[0] = edge
	for i in 1 ..< 3 {
		edge.triA.edges[i] = edgesA[i]
		ed := edgesA[i]
		if ed == nil do continue
		if IsLooseEdge(ed) do non_zero_append(&ctx.pendingDelaunayStack, ed) or_return
		if ed.triA == edge.triA || ed.triB == edge.triA do continue
		if ed.triA == edge.triB do ed.triA = edge.triA
		else if ed.triB == edge.triB do ed.triB = edge.triA
	}

	edge.triB.edges[0] = edge
	for i in 1 ..< 3 {
		edge.triB.edges[i] = edgesB[i]
		ed := edgesB[i]
		if ed == nil do continue
		if IsLooseEdge(ed) do non_zero_append(&ctx.pendingDelaunayStack, ed) or_return
		if ed.triA == edge.triB || ed.triB == edge.triB do continue
		if ed.triA == edge.triA do ed.triA = edge.triB
		else if ed.triB == edge.triA do ed.triB = edge.triB
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

	// find the closest 'active' edge with a vertex that's not above vAbove
	e := ctx.firstActive
	eBelow: ^Edge = nil
	bestD: FixedDef = FixedDef {
		i = -(1 << FIXED_SHIFT),
	}
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
			if eBelow == nil || d.i < fixed.mul(bestD, d_).i {
				eBelow = e
				bestD = d
			}
		}
		e = e.nextE
	}

	if eBelow == nil do return nil, .EBELLOW_MISSING


	// get the best vertex from 'eBelow'
	vBest: ^Vertex = eBelow.vB if eBelow.vT.p.y.i <= yAbove else eBelow.vT
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
MergeDupOrCollinearVertices :: proc(ctx: ^Context) -> (err: Trianguate_Error) {
	// note: this procedure may add new edges and change the
	// number of edges connected with a given vertex, but it
	// won't add or delete vertices (so it's safe to use iterators)
	// Precondition: all_verts must be sorted (y desc, then x asc)
	v1_idx := 0
	for i in 1 ..< len(ctx.all_verts) {
		if !fixed_ex.equal(ctx.all_verts[i].p, ctx.all_verts[v1_idx].p) {
			v1_idx = i
			continue
		}
		vt := &ctx.all_verts[v1_idx]
		v2 := &ctx.all_verts[i]
		if !vt.innerLM || !v2.innerLM {
			vt.innerLM = false
		}

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


		for e, i in vt.e {
			e1 := e
			if IsHorizontal(e1) || e1.vB != vt do continue
			for j := i + 1; j < len(vt.e); j += 1 {
				e2 := vt.e[j]
				if e2 == e1 do continue
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
AddPath :: proc(ctx: ^Context, pts: [][2]FixedDef, poly_idx: int) -> (err: Trianguate_Error) {
	i0 := 0

	// find the first locMin for the current path
	if !FindLocMinIdx(pts, &i0) do return
	iPrev := utils.Prev(i0, len(pts))
	for fixed_ex.equal(pts[iPrev], pts[i0]) {
		iPrev = utils.Prev(iPrev, len(pts))
	}
	iNext := utils.Next(i0, len(pts))

	// it is possible for a locMin here to simply be a
	// collinear spike that should be ignored, so ...
	i := i0
	for CrossProductSign(pts[iPrev], pts[i], pts[iNext]) == 0 {
		FindLocMinIdx(pts, &i) //find again
		if i == i0 do return // this is an entirely collinear path

		iPrev = utils.Prev(i, len(pts))
		for fixed_ex.equal(pts[iPrev], pts[i]) {
			iPrev = utils.Prev(iPrev, len(pts))
		}
		iNext = utils.Next(i, len(pts))
	}


	vert_cnt := len(ctx.all_verts)

	// we are now at the first legitimate locMin
	vert, vert_err := MakeVertex(pts[i])
	if vert_err != nil do return Geometry_To_Triangulate_Error(vert_err)
	non_zero_append(&ctx.all_verts, vert) or_return
	v0 := &ctx.all_verts[vert_cnt]

	is_inner := ctx.polyCCW[poly_idx] == .Clockwise
	v0.innerLM = is_inner
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
			v := &ctx.all_verts[len(ctx.all_verts) - 1]

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
			v := &ctx.all_verts[len(ctx.all_verts) - 1]
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
		vPrev.innerLM = is_inner
	}

	MakeEdge(ctx, v0, vPrev, .descend)


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
AddPaths :: proc(ctx: ^Context) -> (err: Trianguate_Error) {
	total_vert_count := 0
	for p in ctx.polys {
		total_vert_count += len(p)
	}
	if total_vert_count == 0 do return .NO_PATHS
	non_zero_reserve(&ctx.all_verts, cap(ctx.all_verts) + total_vert_count)
	non_zero_reserve(&ctx.all_edges, cap(ctx.all_edges) + total_vert_count)

	for p, i in ctx.polys {
		err := AddPath(ctx, p, i)
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
CreateTriangle :: proc(ctx: ^Context, e1, e2, e3: ^Edge) -> (err: Trianguate_Error) {
	tri := new(Triangle, context.temp_allocator) or_return
	tri.edges = {e1, e2, e3}
	non_zero_append(&ctx.all_tris, tri) or_return
	for i in 0 ..< 3 {
		ed := tri.edges[i]
		if ed.triA != nil {
			ed.triB = tri
			RemoveEdgeFromActives(ctx, ed)
		} else {
			ed.triA = tri
			if ed.kind != .loose do RemoveEdgeFromActives(ctx, ed)
		}
	}
	return
}
