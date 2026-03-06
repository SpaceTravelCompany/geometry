#+feature using-stmt
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
	TOO_MANY_EDGES,
	NO_PATHS,
	FIRST_ACTIVE_MISSING,
	EBELLOW_MISSING,
}

Trianguate_Error :: union #shared_nil {
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
		allocator            = allocator,
		all_verts            = make([dynamic]Vertex, context.temp_allocator) or_return,
		indices              = make([dynamic]u32, context.temp_allocator) or_return,
		all_edges            = make_dynamic_array(
			[dynamic]^Edge,
			context.temp_allocator,
		) or_return,
		pendingDelaunayStack = make([dynamic]^Edge, context.temp_allocator) or_return,
		loc_min_stack        = make([dynamic]^Vertex, context.temp_allocator) or_return,
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
	MergeDupOrCollinearVertices(&ctx)

	currY := ctx.all_verts[0].p.y.i
	//	for

	//TODO non_curves trianglation continue..

	indices = utils.make_non_zeroed_slice([]u32, len(ctx.indices), allocator) or_return
	mem.copy_non_overlapping(
		raw_data(indices),
		raw_data(ctx.indices),
		len(ctx.indices) * size_of(u32),
	)
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
		if e.vL.p.x.i <= xAbove &&
		   e.vR.p.x.i >= xAbove &&
		   e.vB.p.y.i >= yAbove &&
		   e.vB != vAbove &&
		   e.vT != vAbove &&
		   !(CrossProductSign(e.vL.p, vAbove.p, e.vR.p) < 0) {
			d, d_ := ShortestLength2Line(vAbove.p, e.vL.p, e.vR.p)
			if eBelow == nil || d.i < fixed.mul(bestD, d_).i { 	// compare e with eBelow
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
	// todo: fActives is currently *unsorted* but consider making it
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
	return MakeEdge(ctx, vBest, vAbove, .loose)
}


@(private = "file")
MergeDupOrCollinearVertices :: proc(ctx: ^Context) {
	// note: this procedure may add new edges and change the
	// number of edges connected with a given vertex, but it
	// won't add or delete vertices (so it's safe to use iterators)
	vt: ^Vertex
	for i in 1 ..< len(ctx.all_verts) {
		if &ctx.all_verts[i] != vt {
			vt = &ctx.all_verts[i]
			continue
		}

		// merge vt & v2
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
			non_zero_append(&vt.e, e)
		}
		clear(&v2.e)

		for e, i in vt.e {
			e1 := e
			if IsHorizontal(e1, e1) || e1.vB != vt do continue
			for j := i + 1; j < len(vt.e); j += 1 {
				e2 := vt.e[j]
				if e2 == e1 do continue
				if e2.vB != vt || e1.vT.p.y.i == e2.vT.p.y.i || CrossProductSign(e1.vT.p, vt.p, e2.vT.p) != 0 do continue
				// we have parallel edges, both heading up from vt.p.
				// split the longer edge at the top of the shorter edge.
				if e1.vT.p.y.i < e2.vT.p.y.i {
					SplitEdge(ctx, e1, e2)
				} else {
					SplitEdge(ctx, e2, e1)
				}
				break // because only two edges can be collinear
			}
		}
	}
}


@(private = "file")
AddPath :: proc(ctx: ^Context, pts: [][2]FixedDef) -> (err: Trianguate_Error) {
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
	non_zero_append(&ctx.all_verts, MakeVertex(pts[i])) or_return
	v0 := &ctx.all_verts[vert_cnt]

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
			non_zero_append(&ctx.all_verts, MakeVertex(pts[i])) or_return
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
			non_zero_append(&ctx.all_verts, MakeVertex(pts[i]))
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
		if CrossProductSign(vPrev.p, pts[i], pts[iNext]) < 0 do vPrev.innerLM = true
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

