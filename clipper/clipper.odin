package clipper

import "../linalg_ex"
import "base:intrinsics"
import "base:runtime"
import pq "core:container/priority_queue"
import "core:math/fixed"
import "core:slice"
import "shared:utils_private"

FixedDef :: fixed.Fixed(i64, 38)

__Clipper_Error :: enum {
	ADD_LOCAL_MAX_POLY_FAILED,
	FAILED,
}
Clipper_Error :: union #shared_nil {
	__Clipper_Error,
	runtime.Allocator_Error,
}

FillRule :: enum u8 {
	EvenOdd,
	NonZero,
	Positive,
	Negative,
}

ClipType :: enum u8 {
	Union,
	Intersection,
	Difference,
	Xor,
}

@(private = "file")
PathType :: enum u8 {
	Subject,
	Clip,
}

@(private = "file")
JoinWith :: enum u8 {
	NoJoin,
	Left,
	Right,
}

@(private = "file")
VertexFlags :: enum {
	OpenStart,
	OpenEnd,
	LocalMax,
	LocalMin,
}

@(private = "file")
VertexFlags_Set :: bit_set[VertexFlags]

// Clipper context (clipper.engine)
@(private = "file")
Context :: struct {
	bot_y_:              FixedDef,
	actives_:            ^Active,
	sel_:                ^Active,
	minima_list_:        [dynamic]^LocalMinima,
	vertex_lists_:       [dynamic]^Vertex,
	scanline_list_:      pq.Priority_Queue(FixedDef),
	intersect_nodes_:    [dynamic]IntersectNode,
	horz_seg_list_:      [dynamic]HorzSegment,
	horz_join_list_:     [dynamic]HorzJoin,
	out:                 [dynamic][dynamic]FixedDef,
	outrec_list_:        [dynamic]^OutRec,
	current_locmin_idx_: int,
	/*
	Whenever adjacent edges are collinear in closed path solutions,
	the common vertex can be removed from the path without altering its shape.
	However, because some users prefer to retain these vertices, this feature is optional.
	Nevertheless, when adjacent edges in solutions are 180 degree collinear (creating overlapping edge 'spikes'),
	these vertices will be removed irrespective of the PreserveCollinear setting. 
	*/
	preserve_collinear_: bool,
	reverse_solution_:   bool,
	has_open_paths_:     bool,
	fill_rule_:          FillRule,
	clip_type_:          ClipType,
}


// Scanline: y-coordinate for sweep, linked list (clipper.engine.cpp)
@(private = "file")
Scanline :: struct {
	y:    FixedDef,
	next: ^Scanline,
}

// Vertex: polygon vertex, circular doubly linked (clipper.engine.h)
@(private = "file")
Vertex :: struct {
	pt:         [2]FixedDef,
	c0:         [2]FixedDef,
	c1:         [2]FixedDef,
	next:       ^Vertex,
	prev:       ^Vertex,
	flags:      VertexFlags_Set,
	curve_kind: linalg_ex.BezierKind,
}

// LocalMinima: bottom of an edge bound (clipper.engine.h)
@(private = "file")
LocalMinima :: struct {
	vertex:   ^Vertex,
	polytype: PathType,
	is_open:  bool,
}

// OutRec: path in the clipping solution; AEL edges reference it (clipper.engine.h)
@(private = "file")
OutRec :: struct {
	idx:        int,
	owner:      ^OutRec,
	front_edge: ^Active,
	back_edge:  ^Active,
	pts:        ^OutPt,
	is_open:    bool,
}

// OutPt: output polygon vertex, circular doubly linked (clipper.engine.h)
@(private = "file")
OutPt :: struct {
	pt:         [2]FixedDef,
	c0:         [2]FixedDef,
	c1:         [2]FixedDef,
	next:       ^OutPt,
	prev:       ^OutPt,
	outrec:     ^OutRec,
	horz:       ^HorzSegment,
	curve_kind: linalg_ex.BezierKind,
}

// Active: edge in AEL/SEL (clipper.engine.h)
@(private = "file")
Active :: struct {
	bot:           [2]FixedDef,
	top:           [2]FixedDef,
	curr_x:        FixedDef,
	dx:            FixedDef,
	outrec:        ^OutRec,
	prev_in_ael:   ^Active,
	next_in_ael:   ^Active,
	prev_in_sel:   ^Active,
	next_in_sel:   ^Active,
	jump:          ^Active,
	vertex_top:    ^Vertex,
	local_min:     ^LocalMinima,
	wind_dx:       i32, // handle negatives
	wind_cnt:      i32,
	wind_cnt2:     i32,
	is_left_bound: bool,
	join_with:     JoinWith,
}

// IntersectNode: intersection point and two edges (clipper.engine.h)
@(private = "file")
IntersectNode :: struct {
	pt:    [2]FixedDef,
	edge1: ^Active,
	edge2: ^Active,
}

// HorzSegment: horizontal segment for join processing (clipper.engine.h)
@(private = "file")
HorzSegment :: struct {
	left_op:       ^OutPt,
	right_op:      ^OutPt,
	left_to_right: bool,
}

// HorzJoin: horizontal join pair (clipper.engine.h)
@(private = "file")
HorzJoin :: struct {
	op1: ^OutPt,
	op2: ^OutPt,
}

// 단조 분할

@(private = "file")
_fixed_const :: #force_inline proc "contextless" (num: i64) -> FixedDef {
	Frac :: intrinsics.type_polymorphic_record_parameter_value(FixedDef, 1)
	return FixedDef{i = num << Frac}
}

@(private = "file") // TODO 최적화 필요 나중에
_isqrt_u128 :: #force_inline proc "contextless" (n: u128) -> u128 {
	if n == 0 do return 0
	n := n
	res: u128 = 0
	bit: u128 = u128(1) << 126
	for bit > n do bit >>= 2
	for bit != 0 {
		if n >= res + bit {
			n -= res + bit
			res = (res >> 1) + bit
		} else {
			res >>= 1
		}
		bit >>= 2
	}
	return res
}

@(private = "file")
_fixed_sqrt :: #force_inline proc "contextless" (v: FixedDef) -> FixedDef {
	Frac :: intrinsics.type_polymorphic_record_parameter_value(FixedDef, 1)
	scaled := u128(v.i) << Frac
	root := _isqrt_u128(scaled)
	if root > u128(max(i64)) do return FixedDef{i = max(i64)}
	return FixedDef{i = i64(root)}
}

@(private = "file")
SplitEdgeYMonotone :: proc(start_v: ^Vertex) -> (err: Clipper_Error) {
	if start_v == nil || start_v.curve_kind == .Line do return nil
	end_v := start_v.next
	if end_v == nil do return nil

	t_roots: [2]FixedDef
	root_count := 0
	zero := _fixed_const(0)
	one := _fixed_const(1)

	#partial switch start_v.curve_kind {
	case .Quad:
		y0 := start_v.pt.y
		y1 := start_v.c1.y
		y2 := end_v.pt.y
		den := fixed.add(fixed.sub(y0, fixed.mul(_fixed_const(2), y1)), y2)
		if den.i != 0 {
			t := fixed.div(fixed.sub(y0, y1), den)
			if t.i > zero.i && t.i < one.i {
				t_roots[root_count] = t
				root_count += 1
			}
		}
	case .Cubic:
		y0 := start_v.pt.y
		y1 := start_v.c0.y
		y2 := start_v.c1.y
		y3 := end_v.pt.y
		three := _fixed_const(3)
		two := _fixed_const(2)
		four := _fixed_const(4)
		six := _fixed_const(6)

		a := fixed.add(fixed.sub(fixed.mul(three, y1), fixed.mul(three, y2)), fixed.sub(y3, y0))
		b := fixed.add(fixed.sub(fixed.mul(three, y0), fixed.mul(six, y1)), fixed.mul(three, y2))
		c := fixed.sub(fixed.mul(three, y1), fixed.mul(three, y0))

		A := fixed.mul(three, a)
		B := fixed.mul(two, b)
		C := c

		if A.i == 0 {
			if B.i != 0 {
				t := fixed.div(FixedDef{i = -C.i}, B)
				if t.i > zero.i && t.i < one.i {
					t_roots[root_count] = t
					root_count += 1
				}
			}
		} else {
			discriminant := fixed.sub(fixed.mul(B, B), fixed.mul(four, fixed.mul(A, C)))
			if discriminant.i >= 0 {
				sqrt_d := _fixed_sqrt(discriminant)
				den := fixed.mul(two, A)
				if den.i != 0 {
					t0 := fixed.div(fixed.sub(FixedDef{i = -B.i}, sqrt_d), den)
					t1 := fixed.div(fixed.add(FixedDef{i = -B.i}, sqrt_d), den)
					if t0.i > zero.i && t0.i < one.i {
						t_roots[root_count] = t0
						root_count += 1
					}
					if t1.i > zero.i && t1.i < one.i {
						if root_count == 0 || t_roots[0].i != t1.i {
							t_roots[root_count] = t1
							root_count += 1
						}
					}
				}
			}
		}
	}

	if root_count == 0 do return nil
	if root_count == 2 && t_roots[0].i > t_roots[1].i {
		t_roots[0], t_roots[1] = t_roots[1], t_roots[0]
	}

	curr := start_v
	prev_t := zero
	for i in 0 ..< root_count {
		t := t_roots[i]
		denom := fixed.sub(one, prev_t)
		if denom.i == 0 do continue
		t_adj := fixed.div(fixed.sub(t, prev_t), denom)
		if t_adj.i <= zero.i || t_adj.i >= one.i do continue

		new_v := new(Vertex, context.temp_allocator) or_return
		new_v.flags = {}
		new_v.curve_kind = curr.curve_kind

		switch curr.curve_kind {
		case .Quad:
			p01, m, p12 := linalg_ex.SubdivQuadraticBezier(
				[3][2]FixedDef{curr.pt, curr.c1, curr.next.pt},
				t_adj,
			)
			curr.c1 = p01
			new_v.pt = m
			new_v.c1 = p12
		case .Cubic:
			c0, c1, m, d0, d1 := linalg_ex.SubdivCubicBezier(
				[4][2]FixedDef{curr.pt, curr.c0, curr.c1, curr.next.pt},
				t_adj,
			)
			curr.c0 = c0
			curr.c1 = c1
			new_v.pt = m
			new_v.c0 = d0
			new_v.c1 = d1
		case .Line:
			continue
		}

		old_next := curr.next
		curr.next = new_v
		new_v.prev = curr
		new_v.next = old_next
		old_next.prev = new_v

		curr = new_v
		prev_t = t
	}

	return nil
}

@(private = "file") //TODO 나중에 체크
SplitPathCurvesYMonotone :: proc(v0: ^Vertex) -> (err: Clipper_Error) {
	if v0 == nil do return nil
	v := v0
	first := true
	for first || v != v0 {
		first = false
		next_original := v.next
		SplitEdgeYMonotone(v) or_return
		v = next_original
	}
	return nil
}

@(private = "file")
AddSubject :: proc(
	ctx: ^Context,
	subjects: [][][2]FixedDef,
	is_curves: [][]bool = nil,
) -> Clipper_Error {
	if subjects == nil do return nil
	AddPaths(ctx, subjects, .Subject, false, is_curves) or_return
	return nil
}

@(private = "file")
AddOpenSubject :: proc(
	ctx: ^Context,
	opens: [][][2]FixedDef,
	is_curves: [][]bool = nil,
) -> Clipper_Error {
	if opens == nil do return nil
	AddPaths(ctx, opens, .Subject, true, is_curves) or_return
	return nil
}

@(private = "file")
AddClip :: proc(
	ctx: ^Context,
	clips: [][][2]FixedDef,
	is_curves: [][]bool = nil,
) -> Clipper_Error {
	if clips == nil do return nil
	AddPaths(ctx, clips, .Clip, false, is_curves) or_return
	return nil
}

@(private = "file")
AddPaths :: proc(
	ctx: ^Context,
	paths: [][][2]FixedDef,
	polytype: PathType,
	$is_open: bool,
	is_curves: [][]bool = nil,
) -> (
	err: Clipper_Error,
) {
	when is_open do ctx.has_open_paths_ = true
	AddPaths_(ctx, paths, polytype, is_open, is_curves) or_return
	return
}


@(private = "file")
AddPaths_ :: proc(
	ctx: ^Context,
	paths: [][][2]FixedDef,
	polytype: PathType,
	$is_open: bool,
	is_curves: [][]bool = nil,
) -> (
	err: Clipper_Error,
) {
	total_vertices := 0
	for path in paths {
		total_vertices += len(path)
	}
	if total_vertices == 0 do return nil

	all_vertices := (^Vertex)(
		raw_data(
			runtime.mem_alloc_non_zeroed(
				size_of(Vertex) * total_vertices,
				allocator = context.temp_allocator,
			) or_return,
		),
	)

	v := all_vertices
	for path, path_idx in paths {
		//for each path create a circular double linked list of vertices
		v0 := v
		curr_v := v
		prev_v: ^Vertex = nil

		if len(path) == 0 do continue

		has_curves := is_curves != nil && path_idx < len(is_curves)

		cnt := 0
		v.prev = nil
		for pt_idx := 0; pt_idx < len(path); pt_idx += 1 {
			pt := path[pt_idx]
			if prev_v != nil {
				if prev_v.pt == pt do continue // skip duplicates
				prev_v.next = curr_v
			}

			if has_curves {
				cur := is_curves[path_idx]

				if len(cur) > pt_idx + 1 && cur[pt_idx + 1] {
					if len(cur) > pt_idx + 2 && cur[pt_idx + 2] {
						curr_v.curve_kind = .Cubic
						curr_v.c0 = path[pt_idx + 1]
						curr_v.c1 = path[pt_idx + 2]
						pt_idx += 2
					} else {
						curr_v.curve_kind = .Quad
						curr_v.c1 = path[pt_idx + 1]
						pt_idx += 1
					}
				}
			}

			curr_v.prev = prev_v
			curr_v.pt = pt
			curr_v.flags = {}
			prev_v = curr_v
			curr_v = intrinsics.ptr_offset(curr_v, 1)
			cnt += 1
		}
		if prev_v == nil || prev_v.prev == nil do continue
		when !is_open {
			if prev_v.pt == v0.pt do prev_v = prev_v.prev
		}

		prev_v.next = v0
		v0.prev = prev_v
		v = curr_v // get ready for next path

		if cnt < 2 do continue
		when !is_open {
			if cnt == 2 do continue
		}

		// find and assign local minima
		going_up, going_up0: bool
		when is_open {
			curr_v = v0.next
			for curr_v != v0 && (curr_v.pt.y == v0.pt.y) {
				curr_v = curr_v.next
			}

			going_up = curr_v.pt.y.i <= v0.pt.y.i
			if going_up {
				v0.flags = {.OpenStart}
				AddLocMin(ctx, v0, polytype, true) or_return
			} else {
				v0.flags = {.OpenStart, .LocalMax}
			}

		} else { 	// closed path
			prev_v = v0.prev
			for prev_v != v0 && (prev_v.pt.y == v0.pt.y) {
				prev_v = prev_v.prev
			}
			if prev_v == v0 do continue // only open paths can be completely flat
			going_up = prev_v.pt.y.i > v0.pt.y.i
		}

		going_up0 = going_up
		prev_v = v0
		curr_v = v0.next
		for curr_v != v0 {
			if curr_v.pt.y.i > prev_v.pt.y.i && going_up {
				prev_v.flags += {.LocalMax}
				going_up = false
			} else if curr_v.pt.y.i < prev_v.pt.y.i && !going_up {
				going_up = true
				AddLocMin(ctx, prev_v, polytype, is_open) or_return
			}
			prev_v = curr_v
			curr_v = curr_v.next
		}

		when is_open {
			prev_v.flags += {.OpenEnd}
			if going_up do prev_v.flags += {.LocalMax}
			else do AddLocMin(ctx, prev_v, polytype, is_open) or_return
		} else {
			if going_up != going_up0 {
				if going_up0 do AddLocMin(ctx, prev_v, polytype, false) or_return
				else do prev_v.flags += {.LocalMax}
			}
		}
	}
	non_zero_append(&ctx.vertex_lists_, all_vertices) or_return
	return nil
}

@(private = "file")
AddLocMin :: proc(
	ctx: ^Context,
	vert: ^Vertex,
	polytype: PathType,
	$is_open: bool,
) -> (
	err: Clipper_Error,
) {
	if .LocalMin in vert.flags do return nil //make sure the vertex is added only once ...

	vert.flags += {.LocalMin}
	lm := new(LocalMinima, context.temp_allocator) or_return
	lm.vertex = vert
	lm.polytype = polytype
	lm.is_open = is_open
	non_zero_append(&ctx.minima_list_, lm) or_return
	return
}

//convert fixed

@(private = "file")
ToFixedPaths :: proc(
	paths: [][][2]$T,
) -> (
	res: [][][2]FixedDef,
	err: Clipper_Error,
) where intrinsics.type_is_float(T) {
	if paths == nil do return nil, nil
	out := make([dynamic][][2]FixedDef, context.temp_allocator) or_return
	non_zero_resize(&out, len(paths)) or_return
	for i in 0 ..< len(paths) {
		p := paths[i]
		p_out := make([dynamic][2]FixedDef, context.temp_allocator) or_return
		non_zero_resize(&p_out, len(p)) or_return
		for j in 0 ..< len(p) {
			fixed.init_from_f64(&p_out[j].x, f64(p[j].x))
			fixed.init_from_f64(&p_out[j].y, f64(p[j].y))
		}
		out[i] = p_out[:]
	}
	return out[:], nil
}

@(private = "file")
FromFixedPaths :: proc(
	$T: typeid,
	paths: [][][2]FixedDef,
	allocator := context.allocator,
) -> (
	res: [][][2]T,
	err: Clipper_Error,
) where intrinsics.type_is_float(T) {
	if paths == nil do return nil, nil
	out := utils_private.make_non_zeroed_slice([][][2]T, len(paths), allocator) or_return
	for i in 0 ..< len(paths) {
		p := paths[i]
		out[i] = utils_private.make_non_zeroed_slice([][2]T, len(p), allocator) or_return
		for j in 0 ..< len(p) {
			out[i][j] = [2]T{T(fixed.to_f64(p[j].x)), T(fixed.to_f64(p[j].y))}
		}
	}
	return out, nil
}

// Public

BooleanOpCurve_Fixed :: proc(
	clip_type: ClipType,
	subjects: [][][2]FixedDef,
	clips: [][][2]FixedDef,
	opens: [][][2]FixedDef,
	subjects_is_curves: [][]bool,
	clips_is_curves: [][]bool,
	opens_is_curves: [][]bool,
	fill_rule: FillRule = .Positive,
	allocator := context.allocator,
) -> (
	res: [][][2]FixedDef,
	res_open: [][][2]FixedDef,
	res_is_curves: [][]bool,
	res_open_is_curves: [][]bool,
	err: Clipper_Error,
) {
	ctx := Context {
		vertex_lists_       = make([dynamic]^Vertex, context.temp_allocator) or_return,
		minima_list_        = make([dynamic]^LocalMinima, context.temp_allocator) or_return,
		out                 = make([dynamic][dynamic]FixedDef, context.temp_allocator) or_return,
		outrec_list_        = make([dynamic]^OutRec, context.temp_allocator) or_return,
		horz_seg_list_      = make([dynamic]HorzSegment, context.temp_allocator) or_return,
		horz_join_list_     = make([dynamic]HorzJoin, context.temp_allocator) or_return,
		intersect_nodes_    = make([dynamic]IntersectNode, context.temp_allocator) or_return,
		fill_rule_          = fill_rule,
		clip_type_          = clip_type,
		preserve_collinear_ = true,
	}

	pq.init(&ctx.scanline_list_, proc(a, b: FixedDef) -> bool {
			return a.i > b.i
		}, pq.default_swap_proc(FixedDef), allocator = context.temp_allocator) or_return

	AddSubject(&ctx, subjects, subjects_is_curves) or_return
	AddOpenSubject(&ctx, opens, opens_is_curves) or_return
	AddClip(&ctx, clips, clips_is_curves) or_return

	slice.stable_sort_by(ctx.minima_list_[:], proc(a, b: ^LocalMinima) -> bool {
		if b.vertex.pt.y != a.vertex.pt.y do return b.vertex.pt.y.i < a.vertex.pt.y.i
		return b.vertex.pt.x.i > a.vertex.pt.x.i
	})

	#reverse for minima in ctx.minima_list_ {
		InsertScanline(&ctx, minima.vertex.pt.y) or_return
	}

	y: FixedDef
	pop_ok := PopScanline(&ctx, &y) or_return

	if (!pop_ok) do return nil, nil, nil, nil, .FAILED
	for {
		InsertLocalMinimaIntoAEL(&ctx, y) or_return
		e: ^Active
		for {
			e = PopHorz(&ctx)
			if e == nil do break
			DoHorizontal(&ctx, e) or_return
		}
		if len(ctx.horz_seg_list_) > 0 {
			ConvertHorzSegsToJoins(&ctx) or_return
			clear(&ctx.horz_seg_list_)
		}

		ctx.bot_y_ = y
		pop_ok = PopScanline(&ctx, &y) or_return
		if !pop_ok do break

		DoIntersections(&ctx, y) or_return
		DoTopOfScanbeam(&ctx, y) or_return

		for {
			e = PopHorz(&ctx)
			if e == nil do break
			DoHorizontal(&ctx, e) or_return
		}
	}
	ProcessHorzJoins(&ctx) or_return

	// Build result from outrec_list_: CleanCollinear for closed paths, then BuildPath64.
	// nb: outrec_list_.len may change because CleanCollinear can add during FixSelfIntersects.
	solution_close := make([dynamic][dynamic][2]FixedDef, context.temp_allocator) or_return
	solution_open := make([dynamic][dynamic][2]FixedDef, context.temp_allocator) or_return
	non_zero_reserve(&solution_close, len(ctx.outrec_list_)) or_return
	non_zero_reserve(&solution_open, len(ctx.outrec_list_)) or_return

	has_curves := subjects_is_curves != nil || clips_is_curves != nil || opens_is_curves != nil

	solution_close_is_curves: [dynamic][dynamic]bool
	solution_open_is_curves: [dynamic][dynamic]bool
	if has_curves {
		solution_close_is_curves = make([dynamic][dynamic]bool, context.temp_allocator) or_return
		solution_open_is_curves = make([dynamic][dynamic]bool, context.temp_allocator) or_return
		non_zero_reserve(&solution_close_is_curves, len(ctx.outrec_list_)) or_return
		non_zero_reserve(&solution_open_is_curves, len(ctx.outrec_list_)) or_return
	}

	for out in ctx.outrec_list_ {
		if out.pts == nil do continue

		if out.is_open {
			if has_curves {
				BuildPath64(
					out.pts,
					ctx.reverse_solution_,
					true,
					&solution_open,
					&solution_open_is_curves,
				) or_return
			} else {
				BuildPath64(out.pts, ctx.reverse_solution_, true, &solution_open) or_return
			}
		} else {
			CleanCollinear(&ctx, out) or_return
			if has_curves {
				BuildPath64(
					out.pts,
					ctx.reverse_solution_,
					false,
					&solution_close,
					&solution_close_is_curves,
				) or_return
			} else {
				BuildPath64(out.pts, ctx.reverse_solution_, false, &solution_close) or_return
			}
		}
	}

	res, res_is_curves = CopySolutionPathsToSlices(
		has_curves,
		solution_close,
		solution_close_is_curves,
		allocator,
	) or_return
	res_open, res_open_is_curves = CopySolutionPathsToSlices(
		has_curves,
		solution_open,
		solution_open_is_curves,
		allocator,
	) or_return

	return
}

BooleanOp_Fixed :: proc(
	clip_type: ClipType,
	subjects: [][][2]FixedDef,
	clips: [][][2]FixedDef,
	opens: [][][2]FixedDef,
	fill_rule: FillRule = .Positive,
	allocator := context.allocator,
) -> (
	res: [][][2]FixedDef,
	res_open: [][][2]FixedDef,
	err: Clipper_Error,
) {
	res, res_open, _, _ = BooleanOpCurve_Fixed(
		clip_type,
		subjects,
		clips,
		opens,
		nil,
		nil,
		nil,
		fill_rule,
		allocator,
	) or_return
	return
}

BooleanOp :: proc(
	clip_type: ClipType,
	$T: typeid,
	subjects: [][][2]T,
	clips: [][][2]T,
	opens: [][][2]T,
	fill_rule: FillRule = .Positive,
	allocator := context.allocator,
) -> (
	res: [][][2]T,
	res_open: [][][2]T,
	err: Clipper_Error,
) where intrinsics.type_is_float(T) {
	sub_bcd := ToFixedPaths(subjects) or_return
	clip_bcd := ToFixedPaths(clips) or_return
	open_bcd := ToFixedPaths(opens) or_return

	res_bcd, res_open_bcd, _, _ := BooleanOpCurve_Fixed(
		clip_type,
		sub_bcd,
		clip_bcd,
		open_bcd,
		nil,
		nil,
		nil,
		fill_rule,
		context.temp_allocator,
	) or_return

	res = FromFixedPaths(T, res_bcd, allocator) or_return
	res_open = FromFixedPaths(T, res_open_bcd, allocator) or_return
	return
}

BooleanOpCurve :: proc(
	clip_type: ClipType,
	$T: typeid,
	subjects: [][][2]T,
	clips: [][][2]T,
	opens: [][][2]T,
	subjects_is_curves: [][]bool,
	clips_is_curves: [][]bool,
	opens_is_curves: [][]bool,
	fill_rule: FillRule = .Positive,
	allocator := context.allocator,
) -> (
	res: [][][2]T,
	res_open: [][][2]T,
	res_is_curves: [][]bool,
	res_open_is_curves: [][]bool,
	err: Clipper_Error,
) where intrinsics.type_is_float(T) {
	sub_bcd := ToFixedPaths(subjects) or_return
	clip_bcd := ToFixedPaths(clips) or_return
	open_bcd := ToFixedPaths(opens) or_return

	res_fixed: [][][2]FixedDef
	res_open_fixed: [][][2]FixedDef
	res_fixed, res_open_fixed, res_is_curves, res_open_is_curves = BooleanOpCurve_Fixed(
		clip_type,
		sub_bcd,
		clip_bcd,
		open_bcd,
		subjects_is_curves,
		clips_is_curves,
		opens_is_curves,
		fill_rule,
		context.temp_allocator,
	) or_return

	res = FromFixedPaths(T, res_fixed, allocator) or_return
	res_open = FromFixedPaths(T, res_open_fixed, allocator) or_return
	return
}

