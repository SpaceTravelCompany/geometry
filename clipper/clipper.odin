package clipper

import "../linalg_ex"
import "base:intrinsics"
import "base:runtime"
import "core:container/avl"
import pq "core:container/priority_queue"
import "core:math/fixed"
import "shared:utils_private"

FixedDef :: fixed.Fixed(i64, 38)

__Clipper_Error :: enum {
	ADD_LOCAL_MAX_POLY_FAILED,
	FAILED,
	CANT_FIRST_CURVE,
	OPEN_CURVE_END,
	TOO_SMALL,
}
Clipper_Error :: union #shared_nil {
	__Clipper_Error,
	runtime.Allocator_Error,
}

FillRule :: enum u8 {
	EvenOdd,
	NonZero,
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
LeftOrRight :: enum u8 {
	Left,
	Right,
}


// Clipper context (clipper.engine)
@(private = "file")
Context :: struct {
	scanline_list_:  pq.Priority_Queue(^SweepEvent),
	sweep_list_:     avl.Tree(^SweepEvent),
	out:             [dynamic][dynamic]OutPt,
	out_is_open:     [dynamic]bool,
	/*
	Whenever adjacent edges are collinear in closed path solutions,
	the common vertex can be removed from the path without altering its shape.
	However, because some users prefer to retain these vertices, this feature is optional.
	Nevertheless, when adjacent edges in solutions are 180 degree collinear (creating overlapping edge 'spikes'),
	these vertices will be removed irrespective of the PreserveCollinear setting. 
	*/
	//preserve_collinear_: bool,
	//reverse_solution_:   bool,
	has_open_paths_: bool,
	fill_rule_:      FillRule,
	clip_type_:      ClipType,
}


@(private = "file")
SweepEvent :: struct {
	pt:            [2]FixedDef,
	c0:            [2]FixedDef,
	c1:            [2]FixedDef,
	other:         ^SweepEvent,
	winding_cnt:   i32,
	inside:        bool,
	left_or_right: LeftOrRight,
	curve_kind:    linalg_ex.BezierKind,
	pathType:      PathType,
}

@(private = "file")
OutPt :: struct {
	pt:         [2]FixedDef,
	c0:         [2]FixedDef,
	c1:         [2]FixedDef,
	curve_kind: linalg_ex.BezierKind,
}

// 단조 분할

@(private = "file")
_fixed_const :: #force_inline proc "contextless" (num: i64) -> FixedDef {
	Frac :: intrinsics.type_polymorphic_record_parameter_value(FixedDef, 1)
	return FixedDef{i = num << Frac}
}

@(private = "file")
_fixed_sqrt :: #force_inline proc "contextless" (v: FixedDef) -> FixedDef {
	Frac :: intrinsics.type_polymorphic_record_parameter_value(FixedDef, 1)
	scaled := u128(v.i) << Frac
	root :=
		scaled > u128(max(u64)) ? u64(utils_private.sqrt_128(scaled)) : utils_private.sqrt_64(u64(scaled))
	return FixedDef{i = i64(root)}
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
	$polytype: PathType,
	$is_open: bool,
	is_curves: [][]bool = nil,
) -> (
	err: Clipper_Error,
) {
	when is_open do ctx.has_open_paths_ = true

	sweep_line: [2]^SweepEvent
	sweep_line_cnt := 0
	for path, i in paths {
		curves: []bool = nil
		if is_curves != nil && len(is_curves) > i {
			curves = is_curves[i]
			if curves[0] do return .CANT_FIRST_CURVE
		}
		when is_open {
			if len(path) < 2 do return .TOO_SMALL
		} else {
			if len(path) < 3 do return .TOO_SMALL
		}

		j := 0
		closed := false
		for {
			when is_open {
				if j >= len(path) do break
			} else {
				if j >= len(path) {
					j = 0 // 시작점으로 되돌아와서 닫는다.
					closed = true
				} else if closed {
					break
				}
			}
			sweep := utils_private.new_non_zeroed(SweepEvent, context.temp_allocator) or_return
			sweep.pt = path[j]
			sweep.winding_cnt = 0 //? 초기값 필요?
			sweep.inside = false //? 초기값 필요?
			sweep.pathType = polytype

			if curves != nil && !closed { 	// (시작점)끝점인 경우 곡선처리 X
				if j + 1 <= len(curves) - 1 && curves[j + 1] {
					sweep.c0 = path[j]

					if j + 2 <= len(curves) - 1 && curves[j + 2] {
						sweep.c1 = path[j + 1]
						sweep.curve_kind = .Cubic
					} else {
						sweep.curve_kind = .Quad
					}
				} else {
					sweep.curve_kind = .Line
				}
			} else {
				sweep.curve_kind = .Line
			}

			sweep_line[sweep_line_cnt] = sweep
			sweep_line_cnt += 1
			if sweep_line_cnt == 2 {
				sweep_line[0].other = sweep_line[1]
				sweep_line[1].other = sweep_line[0]

				if sweep_line[0].pt.x.i == sweep_line[1].pt.x.i {
					sweep_line[0].left_or_right =
						sweep_line[0].pt.y.i < sweep_line[1].pt.y.i ? .Left : .Right
				} else {
					sweep_line[0].left_or_right =
						sweep_line[0].pt.x.i < sweep_line[1].pt.x.i ? .Left : .Right
				}
				sweep_line[1].left_or_right = sweep_line[0].left_or_right == .Left ? .Right : .Left

				pq.push(&ctx.scanline_list_, sweep_line[0]) or_return
				pq.push(&ctx.scanline_list_, sweep_line[1]) or_return
				sweep_line_cnt = 0
			} else {
				j += 1 // sweep_line_cnt == 2일때는 j를 더하지 않아서 다음 시작점이 이전 끝점과 동일해야 한다.
				if sweep.curve_kind == .Cubic do j += 2
				else if sweep.curve_kind == .Quad do j += 1
			}
		}
	}
	return
}

@(private = "file")
PopScanline :: proc(ctx: ^Context) -> (res: ^SweepEvent, err: Clipper_Error) {
	if pq.len(ctx.scanline_list_) == 0 {
		return nil, nil
	}
	return pq.pop(&ctx.scanline_list_), nil
}

@(private = "file")
BuildPath64 :: proc(
	op: ^[dynamic]OutPt,
	$is_open: bool,
	path: ^[dynamic][dynamic][2]FixedDef,
	path_is_curves: ^[dynamic][dynamic]bool = nil,
) -> Clipper_Error {
	non_zero_append(path, make([dynamic][2]FixedDef, context.temp_allocator) or_return) or_return
	if path_is_curves != nil {
		non_zero_append(
			path_is_curves,
			make([dynamic]bool, context.temp_allocator) or_return,
		) or_return
	}
	//TODO

	insert_pt :: proc(
		op: ^SweepEvent,
		path: ^[dynamic][dynamic][2]FixedDef,
		path_is_curves: ^[dynamic][dynamic]bool,
	) -> Clipper_Error {
		if path_is_curves != nil {
			non_zero_append(&path_is_curves[len(path_is_curves) - 1], false) or_return
		}
		non_zero_append(&path[len(path) - 1], op.pt) or_return

		if op.curve_kind == .Quad {
			non_zero_append(&path_is_curves[len(path_is_curves) - 1], true) or_return
			non_zero_append(&path[len(path) - 1], op.c0) or_return
		} else if op.curve_kind == .Cubic {
			non_zero_append(&path_is_curves[len(path_is_curves) - 1], true) or_return
			non_zero_append(&path_is_curves[len(path_is_curves) - 1], true) or_return
			non_zero_append(&path[len(path) - 1], op.c0) or_return
			non_zero_append(&path[len(path) - 1], op.c1) or_return
		}
		return nil
	}

	return nil
}

@(private = "file")
CopySolutionPathsToSlices :: proc(
	has_curves: bool,
	solution: [dynamic][dynamic][2]FixedDef,
	solution_is_curves: [dynamic][dynamic]bool,
	allocator: runtime.Allocator,
) -> (
	res: [][][2]FixedDef,
	res_is_curves: [][]bool,
	err: Clipper_Error,
) {
	if len(solution) == 0 {
		return nil, nil, nil
	}
	res = utils_private.make_non_zeroed_slice([][][2]FixedDef, len(solution), allocator) or_return
	if has_curves {
		res_is_curves = utils_private.make_non_zeroed_slice(
			[][]bool,
			len(solution),
			allocator,
		) or_return
	}
	for i in 0 ..< len(solution) {
		res[i] = utils_private.make_non_zeroed_slice(
			[][2]FixedDef,
			len(solution[i]),
			allocator,
		) or_return
		if has_curves {
			res_is_curves[i] = utils_private.make_non_zeroed_slice(
				[]bool,
				len(solution_is_curves[i]),
				allocator,
			) or_return
			intrinsics.mem_copy_non_overlapping(
				raw_data(res_is_curves[i]),
				raw_data(solution_is_curves[i]),
				len(solution_is_curves[i]) * size_of(bool),
			)
		}
		intrinsics.mem_copy_non_overlapping(
			raw_data(res[i]),
			raw_data(solution[i]),
			len(solution[i]) * size_of([2]FixedDef),
		)
	}
	return res, res_is_curves, nil
}

@(private = "file")
YatX :: proc(a: ^SweepEvent, x: FixedDef) -> FixedDef {
	return {} //TODO
}

@(private = "file")
SweepList_Cmp :: proc(a, b: ^SweepEvent) -> avl.Ordering {
	ay := YatX(a, a.pt.x)
	by := YatX(b, b.pt.x)

	if ay.i < by.i do return .Less
	else if ay.i > by.i do return .Greater

	if a.other.pt.y.i < b.other.pt.y.i do return .Less
	else if a.other.pt.y.i > b.other.pt.y.i do return .Greater

	return .Equal
}

@(private = "file")
Scanline_Less :: proc(a, b: ^SweepEvent) -> bool {
	if a.pt.x.i == b.pt.x.i {
		if a.pt.y.i == b.pt.y.i {
			if a.left_or_right != b.left_or_right do return a.left_or_right == .Right
			return a.other.pt.y.i < b.other.pt.y.i
		}
		return a.pt.y.i < b.pt.y.i
	}
	return a.pt.x.i < b.pt.x.i
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
	fill_rule: FillRule = .NonZero,
	allocator := context.allocator,
) -> (
	res: [][][2]FixedDef,
	res_open: [][][2]FixedDef,
	res_is_curves: [][]bool,
	res_open_is_curves: [][]bool,
	err: Clipper_Error,
) {
	ctx := Context {
		out         = make([dynamic][dynamic]OutPt, context.temp_allocator) or_return,
		out_is_open = make([dynamic]bool, context.temp_allocator) or_return,
		fill_rule_  = fill_rule,
		clip_type_  = clip_type,
	}

	// 작은 x가 먼저 나옴
	pq.init(
		&ctx.scanline_list_,
		Scanline_Less,
		pq.default_swap_proc(^SweepEvent),
		allocator = context.temp_allocator,
	) or_return

	avl.init_cmp(&ctx.sweep_list_, SweepList_Cmp, context.temp_allocator)

	AddSubject(&ctx, subjects, subjects_is_curves) or_return
	AddOpenSubject(&ctx, opens, opens_is_curves) or_return
	AddClip(&ctx, clips, clips_is_curves) or_return

	y: FixedDef
	pop_res := PopScanline(&ctx) or_return

	if (pop_res == nil) do return nil, nil, nil, nil, .FAILED
	for {

		pop_res = PopScanline(&ctx) or_return
		if pop_res == nil do break
	}

	// Build result from outrec_list_: CleanCollinear for closed paths, then BuildPath64.
	// nb: outrec_list_.len may change because CleanCollinear can add during FixSelfIntersects.
	solution_close := make([dynamic][dynamic][2]FixedDef, context.temp_allocator) or_return
	solution_open := make([dynamic][dynamic][2]FixedDef, context.temp_allocator) or_return

	has_curves := subjects_is_curves != nil || clips_is_curves != nil || opens_is_curves != nil

	solution_close_is_curves: [dynamic][dynamic]bool
	solution_open_is_curves: [dynamic][dynamic]bool
	if has_curves {
		solution_close_is_curves = make([dynamic][dynamic]bool, context.temp_allocator) or_return
		solution_open_is_curves = make([dynamic][dynamic]bool, context.temp_allocator) or_return
	}

	for &out, i in ctx.out {
		if ctx.out_is_open[i] {
			BuildPath64(&out, true, &solution_open, &solution_open_is_curves) or_return
		} else {
			BuildPath64(&out, false, &solution_close, &solution_close_is_curves) or_return
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
	fill_rule: FillRule = .NonZero,
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
	fill_rule: FillRule = .NonZero,
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
	fill_rule: FillRule = .NonZero,
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

