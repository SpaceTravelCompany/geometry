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

__TrianguateError :: enum {
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

TrianguateError :: union #shared_nil {
	__TrianguateError,
	runtime.Allocator_Error,
}

@(private = "file")
Context :: struct {
	allVerts:            [dynamic]^Vertex,
	locMinStack:        [dynamic]^Vertex,
	horzEdgeStack:      [dynamic]^Edge,
	allEdges:            [dynamic]^Edge,
	allTris:             [dynamic]^Triangle,
	pendingDelaunayStack: [dynamic]^Edge,
	indices:              [dynamic]u32,
	polys:                [][][2]f64,
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
	p:       [2]f64,
	e:       [dynamic]^Edge,
	innerLM: bool,
	idx:     u32,
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
	y := v1.p.y
	l, r: f64
	if v1.p.x > v2.p.x {
		l = v2.p.x
		r = v1.p.x
	} else {
		l = v1.p.x
		r = v2.p.x
	}

	res := ctx.firstActive
	for res != nil {
		if res.vL.p.y == y &&
		   res.vR.p.y == y &&
		   res.vL.p.x >= l &&
		   res.vR.p.x <= r &&
		   (res.vL.p.x != l || res.vL.p.x != r) { 	// TODO vR로 바꿔본다.
			break
		}
		res = res.nextE
	}
	return res
}

// Precondition: ctx.all_edges must be sorted ascending on edge.vL.p.x
@(private = "file")
FixupEdgeIntersects :: proc(ctx: ^Context) -> (err: TrianguateError) {
	for i1 in 0 ..< len(ctx.allEdges) {
		e1 := ctx.allEdges[i1]
		for i2 in i1 + 1 ..< len(ctx.allEdges) {
			e2 := ctx.allEdges[i2]
			if e2.vL.p.x >= e1.vR.p.x {
				break
			}
			if e2.vT.p.y < e1.vB.p.y && e2.vB.p.y > e1.vT.p.y {
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
triangleVertices :: proc "contextless" (tri: ^Triangle) -> [3]^Vertex {
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
	minY: f64,
) -> (
	err: TrianguateError,
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
			if (v.p.x > pivot.p.x) == (pivot.p.x > vX.p.x) do continue
		} else if cps > 0 ||
		   (vAlt != nil && !(linalg_ex.CrossProductSign(vX.p, pivot.p, vAlt.p) < 0)) {
			continue
		}

		vAlt = vX
		eAlt = e
	}

	if vAlt == nil || vAlt.p.y < minY do return
	if vAlt.p.y < pivot.p.y {
		if IsLeftEdge(eAlt) do return
	} else if vAlt.p.y > pivot.p.y {
		if IsRightEdge(eAlt) do return
	}

	eX := FindLinkingEdge(vAlt, v, vAlt.p.y < v.p.y)
	if eX == nil {
		if vAlt.p.y == v.p.y && v.p.y == minY && HorizontalBetween(ctx, vAlt, v) != nil {
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
	minY: f64,
) -> (
	err: TrianguateError,
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
			if (v.p.x > pivot.p.x) == (pivot.p.x > vX.p.x) do continue
		} else if cps < 0 ||
		   (vAlt != nil && !(linalg_ex.CrossProductSign(vX.p, pivot.p, vAlt.p) > 0)) { 	// else if right-turning or not the best edge, then continue
			continue
		}
		vAlt = vX
		eAlt = e
	}
	if vAlt == nil || vAlt.p.y < minY do return

	if vAlt.p.y < pivot.p.y {
		if IsRightEdge(eAlt) do return
	} else if vAlt.p.y > pivot.p.y {
		if IsLeftEdge(eAlt) do return
	}

	eX := FindLinkingEdge(vAlt, v, vAlt.p.y > v.p.y)
	if eX == nil {
		if vAlt.p.y == v.p.y && v.p.y == minY && HorizontalBetween(ctx, vAlt, v) != nil {
			return
		}
		eX = MakeEdge(ctx, vAlt, v, .loose) or_return
	}

	CreateTriangle(ctx, edge, eX, eAlt) or_return
	if !EdgeCompleted(eX) do DoTriangulateRight(ctx, eX, vAlt, minY) or_return

	return
}

@(private = "file")
ForceLegal :: proc(ctx: ^Context, edge: ^Edge) -> (err: TrianguateError) {
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
	cross := linalg_ex.CrossProductSign(vertA.p, edge.vL.p, edge.vR.p)
	if cross == 0 do return

	// ictResult - result sign is dependant on triangleA's orientation
	ict := linalg_ex.InCircleTest(vertA.p, edge.vL.p, edge.vR.p, vertB.p)
	if ict == 0 do return
	rightTurn := cross > 0
	if rightTurn == (ict < 0) do return // if on or out of circle then exit

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
	err: TrianguateError,
) {
	if ctx.firstActive == nil do return nil, .FIRST_ACTIVE_MISSING

	xAbove := vAbove.p.x
	yAbove := vAbove.p.y

	e := ctx.firstActive
	eBelow: ^Edge = nil
	bestD, bestD_: f64 = 0.0, 1.0

	for e != nil {
		inRange :=
			e.vL.p.x <= xAbove &&
			e.vR.p.x >= xAbove &&
			e.vB.p.y >= yAbove &&
			e.vB != vAbove &&
			e.vT != vAbove &&
			!(linalg_ex.CrossProductSign(e.vL.p, vAbove.p, e.vR.p) < 0)
		if inRange {
			d, d_ := linalg_ex.ShortestLength2Line(vAbove.p, e.vL.p, e.vR.p)
			if eBelow == nil || d * bestD_ < bestD * d_ {
				eBelow = e
				bestD = d; bestD_ = d_
			}
		}
		e = e.nextE
	}

	if eBelow == nil do return nil, .EBELLOW_MISSING

	vBest: ^Vertex = eBelow.vT.p.y <= yAbove ? eBelow.vB : eBelow.vT
	xBest := vBest.p.x
	yBest := vBest.p.y

	// make sure no edges intersect 'vAbove' and 'vBest' ...
	// TODO: fActives is currently *unsorted* but consider making it
	// a tree structure based on each edge's left and right bounds
	e = ctx.firstActive
	if xBest < xAbove {
		for e != nil {
			if e.vR.p.x > xBest &&
			   e.vL.p.x < xAbove &&
			   e.vB.p.y > yAbove &&
			   e.vT.p.y < yBest &&
			   .intersect == linalg_ex.LinesIntersect3(e.vB.p, e.vT.p, vBest.p, vAbove.p, true) {
				vBest = e.vT.p.y > yAbove ? e.vT : e.vB
				xBest = vBest.p.x
				yBest = vBest.p.y
			}
			e = e.nextE
		}
	} else {
		for e != nil {
			if e.vR.p.x < xBest &&
			   e.vL.p.x > xAbove &&
			   e.vB.p.y > yAbove &&
			   e.vT.p.y < yBest &&
			   .intersect == linalg_ex.LinesIntersect3(e.vB.p, e.vT.p, vBest.p, vAbove.p, true) {
				vBest = e.vT.p.y > yAbove ? e.vT : e.vB
				xBest = vBest.p.x
				yBest = vBest.p.y
			}
			e = e.nextE
		}
	}
	ed = MakeEdge(ctx, vBest, vAbove, .loose) or_return
	return
}


@(private = "file")
MergeDupOrCollinearVertices :: proc(ctx: ^Context) -> (err: TrianguateError) {
	// note: this procedure may add new edges and change the
	// number of edges connected with a given vertex, but it
	// won't add or delete vertices (so it's safe to use iterators)
	// Precondition: all_verts must be sorted (y desc, then x asc)
	v1Idx := 0
	for i in 1 ..< len(ctx.allVerts) {
		if ctx.allVerts[i].p.x != ctx.allVerts[v1Idx].p.x ||
		   ctx.allVerts[i].p.y != ctx.allVerts[v1Idx].p.y {
			v1Idx = i
			continue
		}

		// merge v1 & v2
		vt := ctx.allVerts[v1Idx]
		v2 := ctx.allVerts[i]
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
				if e2.vB != vt || e1.vT.p.y == e2.vT.p.y || linalg_ex.CrossProductSign(e1.vT.p, vt.p, e2.vT.p) != 0 do continue
				// we have parallel edges, both heading up from vt.p.
				// split the longer edge at the top of the shorter edge.
				if e1.vT.p.y < e2.vT.p.y {
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

// Squared distance between two points (same role as Clipper2 DistSqr on path vertices).
@(private = "file")
vertexPairDistSqr :: proc "contextless" (a, b: [2]f64) -> f64 {
	dx := a.x - b.x
	dy := a.y - b.y
	return dx * dx + dy * dy
}


@(private = "file")
AddPath :: proc(ctx: ^Context, pts: [][2]f64, baseIdx: u32) -> (err: TrianguateError) {
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


	vertCnt := len(ctx.allVerts)

	// we are now at the first legitimate locMin
	vert := MakeVertex(pts[i], baseIdx + u32(i)) or_return

	non_zero_append(&ctx.allVerts, vert) or_return
	v0 := ctx.allVerts[vertCnt]
	v0.innerLM = linalg_ex.CrossProductSign(pts[iPrev], pts[i], pts[iNext]) < 0
	vPrev := v0
	i = iNext

	for {
		non_zero_append(&ctx.locMinStack, vPrev) or_return // vPrev is a locMin here

		// update lowermostVertex ...
		if ctx.lowermostVertex == nil ||
		   vPrev.p.y > ctx.lowermostVertex.p.y ||
		   (vPrev.p.y == ctx.lowermostVertex.p.y && vPrev.p.x < ctx.lowermostVertex.p.x) {
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

		for pts[i].y <= vPrev.p.y { 	// ascend up next bound to LocMax
			vert := MakeVertex(pts[i], baseIdx + u32(i)) or_return

			non_zero_append(&ctx.allVerts, vert) or_return
			v := ctx.allVerts[len(ctx.allVerts) - 1]

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
		for i != i0 && pts[i].y >= vPrev.p.y {
			vert := MakeVertex(pts[i], baseIdx + u32(i)) or_return
			non_zero_append(&ctx.allVerts, vert) or_return
			v := ctx.allVerts[len(ctx.allVerts) - 1]
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

	// Ignore path if not a polygon
	lenFin := len(ctx.allVerts) - vertCnt
	ignorePath := lenFin < 3
	if !ignorePath && lenFin == 3 {
		p0 := ctx.allVerts[vertCnt + 0].p
		p1 := ctx.allVerts[vertCnt + 1].p
		p2 := ctx.allVerts[vertCnt + 2].p
	}
	if ignorePath {
		for j := vertCnt; j < len(ctx.allVerts); j += 1 {
			clear(&ctx.allVerts[j].e)
		}
	}

	return
}

@(private = "file")
AddPaths :: proc(ctx: ^Context) -> (err: TrianguateError) {
	totalVertCount := 0
	for p in ctx.polys {
		totalVertCount += len(p)
	}
	if totalVertCount == 0 do return .NO_PATHS
	non_zero_reserve(&ctx.allVerts, cap(ctx.allVerts) + totalVertCount)
	non_zero_reserve(&ctx.allEdges, cap(ctx.allEdges) + totalVertCount)

	baseIdx: u32 = 0
	for p in ctx.polys {
		err := AddPath(ctx, p, baseIdx)
		if err != nil && err != .NO_PATHS do return err
		baseIdx += u32(len(p))
	}
	if len(ctx.allVerts) <= 2 do return .NO_PATHS
	return
}

@(private = "file")
FindLocMinIdx :: proc "contextless" (pts: [][2]f64, inOutIdx: ^int) -> bool {
	if len(pts) < 3 do return false

	i0 := inOutIdx^
	idx := inOutIdx
	next := utils.Next(idx^, len(pts))
	for ; pts[next].y <= pts[idx^].y; next = utils.Next(next, len(pts)) {
		idx^ = next
		if idx^ == i0 do return false // fails if path is completely horizontal
	}

	for pts[next].y >= pts[idx^].y {
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
	err: TrianguateError,
) {
	res = new(Triangle, context.temp_allocator) or_return
	res.edges = {e1, e2, e3}
	non_zero_append(&ctx.allTris, res) or_return
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
RemoveEdgeFromVertex :: proc(vert: ^Vertex, edge: ^Edge) -> (err: TrianguateError) {
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
	// e.vB.p.y <= e.vT.p.y + linalg_ex.epsilon(f64) && e.vB.p.y >= e.vT.p.y - linalg_ex.epsilon(f64)
	return e.vB.p.y == e.vT.p.y
}

@(private = "file")
MakeEdge :: proc(
	ctx: ^Context,
	v1, v2: ^Vertex,
	kind: EdgeKind,
) -> (
	res: ^Edge,
	err: TrianguateError,
) {
	when size_of(int) > 4 { 	//only 64bits can overflow len(int) than max(u32)
		if len(ctx.allEdges) >= int(max(u32)) do return nil, .TOO_MANY_EDGES
	}
	non_zero_append(&ctx.allEdges, new(Edge, context.temp_allocator) or_return) or_return

	ed := ctx.allEdges[len(ctx.allEdges) - 1]
	if v1.p.y >= v2.p.y {
		ed.vB = v1; ed.vT = v2
	} else {
		ed.vB = v2; ed.vT = v1
	}

	if v1.p.x <= v2.p.x {
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
RemoveEdgeFromActives :: proc(ctx: ^Context, edge: ^Edge) -> (err: TrianguateError) {
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
SplitEdge :: proc(ctx: ^Context, longE, shortE: ^Edge) -> (err: TrianguateError) {
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
RemoveIntersection :: proc(ctx: ^Context, e1: ^Edge, e2: ^Edge) -> (err: TrianguateError) {
	v: ^Vertex = e1.vL
	tmpE: ^Edge = e2

	d, d_ := linalg_ex.ShortestLength2Line(e1.vL.p, e2.vL.p, e2.vR.p)
	d2, d2_ := linalg_ex.ShortestLength2Line(e1.vR.p, e2.vL.p, e2.vR.p)
	if d2 / d2_ < d / d_ {
		d = d2; d_ = d2_
		v = e1.vR
	}

	d2, d2_ = linalg_ex.ShortestLength2Line(e2.vL.p, e1.vL.p, e1.vR.p)
	if d2 / d2_ < d / d_ {
		d = d2; d_ = d2_
		tmpE = e1
		v = e2.vL
	}

	d2, d2_ = linalg_ex.ShortestLength2Line(e2.vR.p, e1.vL.p, e1.vR.p)
	if d2 / d2_ < d / d_ {
		d = d2; d_ = d2_
		tmpE = e1
		v = e2.vR
	}
	if d / d_ > 1.0 {
		return .PATHS_INTERSECTS
	}

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
	asc, des: int
	if v.e[0].kind == .ascend {asc = 0; des = 1} else {des = 0; asc = 1}
	return linalg_ex.GetAngle(v.e[des].vT.p, v.p, v.e[asc].vT.p) <= 0.0
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
FindLinkingEdge :: proc "contextless" (
	vert1: ^Vertex,
	vert2: ^Vertex,
	preferAscending: bool,
) -> ^Edge {
	res: ^Edge = nil
	for e in vert1.e {
		if e.vL == vert2 || e.vR == vert2 {
			if e.kind == .loose || (e.kind == .ascend) == preferAscending do return e
			res = e
		}
	}
	return res
}

@(private = "file")
MakeVertex :: proc(p: [2]f64, idx: u32) -> (res: ^Vertex, err: TrianguateError) {
	res = new_clone(Vertex{p = p, innerLM = false}, context.temp_allocator) or_return
	res.e = make([dynamic]^Edge, context.temp_allocator) or_return
	res.idx = idx
	non_zero_reserve(&res.e, 2) or_return
	return
}

TrianguatePolygons :: proc(
	poly: [][][2]$T,
	allocator := context.allocator,
	offset: u32 = 0,
) -> (
	indices: []u32,
	err: TrianguateError,
) where intrinsics.type_is_float(T) {
	poly64 := make([][][2]f64, len(poly), context.temp_allocator) or_return
	for p, i in poly {
		poly64[i] = make([][2]f64, len(p), context.temp_allocator) or_return
		for pp, j in p {
			poly64[i][j] = [2]f64{f64(pp.x), f64(pp.y)}
		}
	}

	ctx := Context {
		allVerts            = make([dynamic]^Vertex, context.temp_allocator) or_return,
		allTris             = make([dynamic]^Triangle, context.temp_allocator) or_return,
		allEdges            = make([dynamic]^Edge, context.temp_allocator) or_return,
		pendingDelaunayStack = make([dynamic]^Edge, context.temp_allocator) or_return,
		locMinStack        = make([dynamic]^Vertex, context.temp_allocator) or_return,
		horzEdgeStack      = make([dynamic]^Edge, context.temp_allocator) or_return,
		polys                = poly64,
	}
	non_zero_reserve(&ctx.allEdges, len(poly)) or_return

	AddPaths(&ctx) or_return

	if ctx.lowermostVertex.innerLM {
		lm: ^Vertex
		for len(ctx.locMinStack) > 0 {
			lm = ctx.locMinStack[len(ctx.locMinStack) - 1]
			lm.innerLM = !lm.innerLM
			non_zero_resize(&ctx.locMinStack, len(ctx.locMinStack) - 1) or_return
		}

		for &e in ctx.allEdges {
			if e.kind == .ascend do e.kind = .descend
			else do e.kind = .ascend
		}
	} else {
		clear(&ctx.locMinStack)
	}

	slice.sort_by(ctx.allEdges[:], proc(a, b: ^Edge) -> bool {
		return a.vL.p.x < b.vL.p.x
	})
	FixupEdgeIntersects(&ctx) or_return

	slice.sort_by(ctx.allVerts[:], proc(a, b: ^Vertex) -> bool {
		if a.p.y == b.p.y do return a.p.x < b.p.x
		return a.p.y > b.p.y
	})
	MergeDupOrCollinearVertices(&ctx) or_return

	currY: f64 = ctx.allVerts[0].p.y
	for &v in ctx.allVerts {
		if len(v.e) == 0 do continue

		if v.p.y != currY {
			// JOIN AN INNER LOCMIN WITH A SUITABLE EDGE BELOW
			for len(ctx.locMinStack) > 0 {
				lm := ctx.locMinStack[len(ctx.locMinStack) - 1]
				non_zero_resize(&ctx.locMinStack, len(ctx.locMinStack) - 1) or_return

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

			for len(ctx.horzEdgeStack) > 0 {
				e := ctx.horzEdgeStack[len(ctx.horzEdgeStack) - 1]
				non_zero_resize(&ctx.horzEdgeStack, len(ctx.horzEdgeStack) - 1)
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
					non_zero_append(&ctx.horzEdgeStack, e) or_return
				}
				// locMin 엣지는 actives 추가를 지연
				if !v.innerLM {
					SetEdgeToActive(&ctx, e)
				}
			} else {
				if IsHorizontal(e) {
					non_zero_append(&ctx.horzEdgeStack, e) or_return
				} else if IsLeftEdge(e) {
					DoTriangulateLeft(&ctx, e, e.vB, v.p.y) or_return
				} else {
					DoTriangulateRight(&ctx, e, e.vB, v.p.y) or_return
				}
			}
		} // inner edge loop

		if v.innerLM {
			non_zero_append(&ctx.locMinStack, v) or_return
		}
	} // for all_verts

	// 버텍스 루프 후 남은 수평 엣지 처리
	for len(ctx.horzEdgeStack) > 0 {
		e := ctx.horzEdgeStack[len(ctx.horzEdgeStack) - 1]
		non_zero_resize(&ctx.horzEdgeStack, len(ctx.horzEdgeStack) - 1) or_return

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
	idxBuf := make([dynamic]u32, context.temp_allocator) or_return
	defer delete(idxBuf)
	for tri in ctx.allTris {
		vs := triangleVertices(tri)
		p0, p1, p2 := vs[0].p, vs[1].p, vs[2].p
		cps := linalg_ex.CrossProductSign(p0, p1, p2)
		if cps == 0 do continue // skip degenerate triangles
		i0 := vs[0].idx
		i1 := vs[1].idx
		i2 := vs[2].idx
		if cps < 0 {
			// CCW -> reverse to CW
			i1, i2 = i2, i1
		}
		non_zero_append(&idxBuf, i0 + offset, i1 + offset, i2 + offset) or_return
	}
	indices = utils.makeNonZeroedSlice([]u32, len(idxBuf), allocator) or_return
	mem.copy_non_overlapping(raw_data(indices), raw_data(idxBuf[:]), size_of(u32) * len(idxBuf))
	return
}
