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
}

Trianguate_Error :: union #shared_nil {
    __Trianguate_Error,
    runtime.Allocator_Error,
}

@(private = "file") Edge :: struct {
	vL:     ^Vertex,
	vR:     ^Vertex,
	vB:     ^Vertex,
	vT:     ^Vertex,
	triA:   ^Triangle,
	triB:   ^Triangle,
	nextE:  ^Edge,
	prevE:  ^Edge,
	kind:   EdgeKind,
	isActive: bool,
}

@(private = "file") Triangle :: struct {
	edges: [3]^Edge,
}

@(private = "file") EdgeKind :: enum u8 { loose, ascend, descend } // ascend & descend are 'fixed' edges
@(private = "file") IntersectKind :: enum u8 { none, collinear, intersect }
@(private = "file") EdgeContainsResult :: enum u8 { neither, left, right }

@(private = "file") Vertex :: struct {
	p: [2]FixedDef,
	e: [dynamic]^Edge,
	innerLM: bool,//check inner local minimum
}

@(private = "file") Context :: struct {
	all_verts: [dynamic]Vertex,
	loc_min_stack: [dynamic]^Vertex,
	all_edges: [dynamic]^Edge,
	pendingDelaunayStack: [dynamic]^Edge,
	indices: [dynamic]u32,
	polys:[][][2]FixedDef,
	polyCCW: []PolyOrientation,
	allocator: runtime.Allocator,
	lowermostVertex: ^Vertex,
	firstActive: ^Edge,
}

TrianguatePolygons_Fixed :: proc(poly:[][][2]FixedDef, polyCCW: []PolyOrientation = nil, allocator := context.allocator) ->
(indices:[]u32, err:Trianguate_Error) {
	ctx := Context {
		allocator = allocator,
		all_verts = make([dynamic]Vertex, context.temp_allocator) or_return,
		indices = make([dynamic]u32, context.temp_allocator) or_return,
		all_edges = make_dynamic_array([dynamic]^Edge, context.temp_allocator) or_return,
		pendingDelaunayStack = make([dynamic]^Edge, context.temp_allocator) or_return,
		loc_min_stack = make([dynamic]^Vertex, context.temp_allocator) or_return,
		polys = poly,
	}

	ctx.polyCCW = polyCCW
	if ctx.polyCCW == nil {//polyCCW 정보 (폴리곤이 구멍인지 아닌지 여부) 가 없으면 직접 만들어야 한다.
		ctx.polyCCW = utils.make_non_zeroed_slice([]PolyOrientation, len(poly), context.temp_allocator) or_return
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
		lm : ^Vertex
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
	
	
	//TODO non_curves trianglation continue..
	
	indices = utils.make_non_zeroed_slice([]u32, len(ctx.indices), allocator) or_return
	mem.copy_non_overlapping(raw_data(indices), raw_data(ctx.indices), len(ctx.indices) * size_of(u32))
	return
}

@(private = "file") MakeVertex :: proc(p:[2]FixedDef) -> Vertex {
	res := Vertex {
		p = p,
		innerLM = false,
	}
	res.e = make([dynamic]^Edge, context.temp_allocator)
	non_zero_reserve(&res.e, 2)
	return res
}

@(private = "file") MakeEdge :: proc(ctx: ^Context, v1, v2: ^Vertex, kind: EdgeKind) -> (res:^Edge, err:Trianguate_Error) {
	non_zero_append(&ctx.all_edges, new(Edge, context.temp_allocator)) or_return
	if len(&ctx.all_edges) >= int(max(u32) - 1) do return nil, .TOO_MANY_EDGES// - 1 because max(u32) uses nil

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

	return
}

@(private = "file") SetEdgeToActive :: proc (ctx: ^Context, edge:^Edge) {
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

@(private = "file") AddPath :: proc (ctx: ^Context, pts:[][2]FixedDef) -> (err:Trianguate_Error) {
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
		FindLocMinIdx(pts, &i)//find again
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
		non_zero_append(&ctx.loc_min_stack, vPrev) or_return// vPrev is a locMin here

		// update lowermostVertex ...
		if ctx.lowermostVertex == nil || vPrev.p.y.i > ctx.lowermostVertex.p.y.i ||
		(vPrev.p.y.i == ctx.lowermostVertex.p.y.i && vPrev.p.x.i < ctx.lowermostVertex.p.x.i) {
			ctx.lowermostVertex = vPrev
		}

		iNext = utils.Next(i, len(pts))
		if CrossProductSign(vPrev.p, pts[i], pts[iNext]) == 0 {//skips collinear path (vPrev, pts[i], pts[iNext] is straight)
			i = iNext
			continue
		}

		for pts[i].y.i <= vPrev.p.y.i {// ascend up next bound to LocMax
			non_zero_append(&ctx.all_verts, MakeVertex(pts[i])) or_return
			v := &ctx.all_verts[len(ctx.all_verts)-1]

			MakeEdge(ctx, vPrev, v, .ascend)
			vPrev = v
			i = iNext

			for iNext = utils.Next(i, len(pts)); CrossProductSign(vPrev.p, pts[i], pts[iNext]) == 0; {
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

			for iNext = utils.Next(i, len(pts)); CrossProductSign(vPrev.p, pts[i], pts[iNext]) == 0; {
				i = iNext
				iNext = utils.Next(i, len(pts))
			}
		}

		if i == i0 do break
		if CrossProductSign(vPrev.p, pts[i], pts[iNext]) < 0 do vPrev.innerLM = true
	}

	MakeEdge(ctx, v0, vPrev, .descend)

	len_fin := len(ctx.all_verts) - vert_cnt// finally, ignore this path if is not a polygon or too small
	i = vert_cnt
	if len_fin < 3 {//Skipped the original polygon-size check for now — needs testing.
		for j := vert_cnt; j < len(ctx.all_verts); j += 1 {
			clear(&ctx.all_verts[j].e) // flag to ignore //TODO 갯수만 0개로 하면 되는지 다음에 확인해보자
		}
	}

	return
}

@(private = "file") AddPaths :: proc (ctx: ^Context) -> (err:Trianguate_Error) {
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

@(private = "file") FindLocMinIdx :: proc "contextless" (pts:[][2]FixedDef, in_out_idx:^int) -> bool {
	if len(pts) < 3 do return false

	i0 := in_out_idx^
	idx := in_out_idx
	next := utils.Next(idx^, len(pts))
	for ;pts[next].y.i <= pts[idx^].y.i; next = utils.Next(next, len(pts)) {
		idx^ = next
		if idx^ == i0 do return false  // fails if path is completely horizontal
	}

	for pts[next].y.i >= pts[idx^].y.i {
		idx^ = next
		next = utils.Next(next, len(pts))
	}

	return true
}
