//!incomplete
package clipper

import "../linalg_ex"
import "base:intrinsics"
import "base:runtime"
import "core:container/avl"
import pq "core:container/priority_queue"
import "core:math/fixed"
import "engine:utils_private"

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
	scanline_list_:  pq.Priority_Queue(SweepEventNode),
	sweep_list_:     avl.Tree(^SweepEvent),
	out:             [dynamic][dynamic]OutPt,
	out_is_open:     [dynamic]bool,
	sweep_x:         FixedDef,
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
	other:         [2]FixedDef,
	winding_cnt:   i32,
	inside:        bool,
	left_or_right: LeftOrRight,
	curve_kind:    linalg_ex.BezierKind,
	pathType:      PathType,
}

@(private = "file")
SweepEventNode :: struct {
	sweep:  ^SweepEvent,
	is_end: bool,
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
SetLeftOrRight :: proc "contextless" (sweep: ^SweepEvent) {
	if sweep.pt.x.i == sweep.other.x.i {
		sweep.left_or_right = sweep.pt.y.i < sweep.other.y.i ? .Left : .Right
	} else {
		sweep.left_or_right = sweep.pt.x.i < sweep.other.x.i ? .Left : .Right
	}
}

@(private = "file")
NewSweep :: proc(
	pt: [2]FixedDef,
	path_type: PathType,
) -> (
	sweep: ^SweepEvent,
	err: Clipper_Error,
) {
	sweep = utils_private.new_non_zeroed(SweepEvent, context.temp_allocator) or_return
	sweep.pt = pt
	sweep.winding_cnt = 0 //? 초기값 필요?
	sweep.inside = false //? 초기값 필요?
	sweep.pathType = path_type

	return
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
		for {
			when is_open {
				if j >= len(path) - 1 do break //맨끝 전에서 이미 끝을 other로 참조한다.
			} else {
				if j >= len(path) {
					break
				}
			}
			sweep := NewSweep(path[j], polytype) or_return

			if curves != nil { 	//TODO 이웃한 점이 좌표가 완전히 겹칠때 처리? 일단 생각안함.
				if j + 1 <= len(curves) - 1 && curves[j + 1] {
					sweep.c0 = path[j]
					if j + 2 <= len(curves) - 1 && curves[j + 2] {
						sweep.c1 = path[j + 1]
						sweep.curve_kind = .Cubic
						j += 2
					} else {
						sweep.curve_kind = .Quad
						j += 1
					}
				} else {
					sweep.curve_kind = .Line
				}
			} else {
				sweep.curve_kind = .Line
			}

			j += 1
			if j == len(path) {
				sweep.other = path[0]
			} else {
				sweep.other = path[j]
			}
			ONE: FixedDef.Backing : (1 << FixedDef.Fraction_Width)
			if sweep.curve_kind == .Quad {
				t, _ := linalg_ex.GetBezierTForXMonotone(
					.Quad,
					[4]FixedDef{sweep.pt.x, sweep.c0.x, sweep.other.x, {}},
				)
				if t.i > 0 && t.i < ONE {
					p1, p12, p2 := linalg_ex.SubdivQuadraticBezier(
						[3][2]FixedDef{sweep.pt, sweep.c0, sweep.other},
						t,
					)
					end := sweep.other
					sweep.c0 = p1
					sweep.other = p12
					SetLeftOrRight(sweep)
					pq.push(&ctx.scanline_list_, SweepEventNode{sweep, false}) or_return
					pq.push(&ctx.scanline_list_, SweepEventNode{sweep, true}) or_return

					sweep = NewSweep(sweep.other, polytype) or_return
					sweep.curve_kind = .Quad
					sweep.c0 = p2
					sweep.other = end
				}
			} else if sweep.curve_kind == .Cubic {
				t0, t1 := linalg_ex.GetBezierTForXMonotone(
					.Cubic,
					[4]FixedDef{sweep.pt.x, sweep.c0.x, sweep.c1.x, sweep.other.x},
				)
				if t0.i > 0 && t0.i < ONE && (t1.i <= 0 || t1.i >= ONE) { 	// t0만 유효할경우 한번만 나눈다.
					c0, c1, m, d0, d1 := linalg_ex.SubdivCubicBezier(
						[4][2]FixedDef{sweep.pt, sweep.c0, sweep.c1, sweep.other},
						t0,
					)
					end := sweep.other

					sweep.c0 = c0
					sweep.c1 = c1
					sweep.other = m
					SetLeftOrRight(sweep)
					pq.push(&ctx.scanline_list_, SweepEventNode{sweep, false}) or_return
					pq.push(&ctx.scanline_list_, SweepEventNode{sweep, true}) or_return

					sweep = NewSweep(sweep.other, polytype) or_return
					sweep.curve_kind = .Cubic
					sweep.c0 = d0
					sweep.c1 = d1
					sweep.other = end
				} else if t0.i > 0 && t0.i < ONE && t1.i > 0 && t1.i < ONE { 	// 둘다 유효하면 삼등분
					c0, c1, m, d0, d1 := linalg_ex.SubdivCubicBezier(
						[4][2]FixedDef{sweep.pt, sweep.c0, sweep.c1, sweep.other},
						t0,
					)
					end := sweep.other

					sweep.c0 = c0
					sweep.c1 = c1
					sweep.other = m
					SetLeftOrRight(sweep)
					pq.push(&ctx.scanline_list_, SweepEventNode{sweep, false}) or_return
					pq.push(&ctx.scanline_list_, SweepEventNode{sweep, true}) or_return

					t1 = fixed.div(fixed.sub(t1, t0), fixed.sub(FixedDef{i = ONE}, t0))
					cc0, cc1, mm, dd0, dd1 := linalg_ex.SubdivCubicBezier(
						[4][2]FixedDef{m, d0, d1, end},
						t1,
					)

					sweep = NewSweep(sweep.other, polytype) or_return
					sweep.curve_kind = .Cubic
					sweep.c0 = cc0
					sweep.c1 = cc1
					sweep.other = mm
					SetLeftOrRight(sweep)
					pq.push(&ctx.scanline_list_, SweepEventNode{sweep, false}) or_return
					pq.push(&ctx.scanline_list_, SweepEventNode{sweep, true}) or_return

					sweep = NewSweep(sweep.other, polytype) or_return
					sweep.curve_kind = .Cubic
					sweep.c0 = dd0
					sweep.c1 = dd1
					sweep.other = end
				}
			}

			SetLeftOrRight(sweep)
			pq.push(&ctx.scanline_list_, SweepEventNode{sweep, false}) or_return
			pq.push(&ctx.scanline_list_, SweepEventNode{sweep, true}) or_return
		}
	}
	return
}

@(private = "file")
PopScanline :: proc(ctx: ^Context) -> (res: SweepEventNode, err: Clipper_Error) {
	if pq.len(ctx.scanline_list_) == 0 {
		return {}, nil
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
	ONE :: FixedDef {
		i = 1 << FixedDef.Fraction_Width,
	}
	TWO :: FixedDef {
		i = 2 << FixedDef.Fraction_Width,
	}
	THREE :: FixedDef {
		i = 3 << FixedDef.Fraction_Width,
	}

	if a.curve_kind == .Line {
		dx := fixed.sub(a.pt.x, a.other.x)
		if dx.i == 0 do return a.pt.y.i < a.other.y.i ? a.pt.y : a.other.y
		return fixed.add(
			a.pt.y,
			fixed.mul(fixed.sub(x, a.pt.x), fixed.div(fixed.sub(a.other.y, a.pt.y), dx)),
		)
	} else if a.curve_kind == .Quad {
		p0 := a.pt
		p1 := a.c0
		p2 := a.other

		// t = (x - p0.x) / (p2.x - p0.x)
		t := fixed.div(fixed.sub(x, p0.x), fixed.sub(p2.x, p0.x))
		mt := fixed.sub(ONE, t)

		mt2 := fixed.mul(mt, mt)
		tmt := fixed.mul(t, mt)
		t2 := fixed.mul(t, t)

		// y = (1-t)²·p0.y + 2·t·(1-t)·p1.y + t²·p2.y
		return fixed.add(
			fixed.add(fixed.mul(mt2, p0.y), fixed.mul(fixed.mul(TWO, tmt), p1.y)),
			fixed.mul(t2, p2.y),
		)
	} else {
		p0 := a.pt
		p1 := a.c0
		p2 := a.c1
		p3 := a.other

		// t = (x - p0.x) / (p3.x - p0.x)
		t := fixed.div(fixed.sub(x, p0.x), fixed.sub(p3.x, p0.x))
		mt := fixed.sub(ONE, t)

		mt2 := fixed.mul(mt, mt)
		mt3 := fixed.mul(mt2, mt)
		tmt2 := fixed.mul(t, mt2)
		t2mt := fixed.mul(fixed.mul(t, t), mt)
		t2 := fixed.mul(t, t)
		t3 := fixed.mul(t2, t)

		// y = (1-t)³·p0.y + 3·t·(1-t)²·p1.y + 3·t²·(1-t)·p2.y + t³·p3.y
		return fixed.add(
			fixed.add(
				fixed.add(fixed.mul(mt3, p0.y), fixed.mul(fixed.mul(THREE, tmt2), p1.y)),
				fixed.mul(fixed.mul(THREE, t2mt), p2.y),
			),
			fixed.mul(t3, p3.y),
		)
	}
	return {}
}

@(private = "file")
SweepList_Cmp :: proc(a, b: ^SweepEvent) -> avl.Ordering {
	ctx: ^Context = (^Context)(context.user_ptr)
	ay := YatX(a, ctx.sweep_x)
	by := YatX(b, ctx.sweep_x)

	if ay.i < by.i do return .Less
	else if ay.i > by.i do return .Greater

	if a.other.y.i < b.other.y.i do return .Less
	else if a.other.y.i > b.other.y.i do return .Greater

	return .Equal
}

@(private = "file")
Scanline_Less :: proc(a, b: SweepEventNode) -> bool {
	aa := a.sweep.pt
	bb := b.sweep.pt
	ao := a.sweep.other
	bo := b.sweep.other
	if a.is_end do aa, ao = ao, aa
	if b.is_end do bb, bo = bo, bb

	if aa.x.i == bb.x.i {
		if aa.y.i == bb.y.i {
			if a.sweep.left_or_right != b.sweep.left_or_right do return a.sweep.left_or_right == .Right
			return ao.y.i < bo.y.i
		}
		return aa.y.i < bb.y.i
	}
	return aa.x.i < bb.x.i
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
@(private = "file")
possibleInter :: proc(a, b: ^SweepEvent) {
	if a == b do return

	if a.curve_kind == .Line && b.curve_kind == .Line {
		kind, pt := linalg_ex.LinesIntersect2(a.pt, a.other, b.pt, b.other, true)
		if kind != .intersect do return
		//TODO
	}
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
	context.user_ptr = &ctx

	// 작은 x가 먼저 나옴
	pq.init(
		&ctx.scanline_list_,
		Scanline_Less,
		pq.default_swap_proc(SweepEventNode),
		allocator = context.temp_allocator,
	) or_return

	avl.init_cmp(&ctx.sweep_list_, SweepList_Cmp, context.temp_allocator)

	AddSubject(&ctx, subjects, subjects_is_curves) or_return
	AddOpenSubject(&ctx, opens, opens_is_curves) or_return
	AddClip(&ctx, clips, clips_is_curves) or_return

	pop_res := PopScanline(&ctx) or_return

	if (pop_res.sweep == nil) do return nil, nil, nil, nil, .FAILED
	for {
		ctx.sweep_x = pop_res.sweep.pt.x
		node, inserted := avl.find_or_insert(&ctx.sweep_list_, pop_res.sweep) or_return

		if inserted { 	//처음 들어갔을때 첫번째 교차 검사
			it_next: avl.Iterator(^SweepEvent) = avl.iterator_from_pos(
				&ctx.sweep_list_,
				node,
				.Forward,
			)
			it_prev: avl.Iterator(^SweepEvent) = avl.iterator_from_pos(
				&ctx.sweep_list_,
				node,
				.Backward,
			)

			if it_next._next != nil {
				possibleInter(node.value, it_next._next.value)
			}
			if it_prev._next != nil {
				possibleInter(node.value, it_prev._next.value)
			}
		} else { 	//이미 들어간 것이면 해당 변 제거후 그자리에서 다시 교차 검사
			it_next: avl.Iterator(^SweepEvent) = avl.iterator_from_pos(
				&ctx.sweep_list_,
				node,
				.Forward,
			)
			it_prev: avl.Iterator(^SweepEvent) = avl.iterator_from_pos(
				&ctx.sweep_list_,
				node,
				.Backward,
			)
			next, prev: ^SweepEvent
			if it_next._next != nil && it_prev._next != nil {
				next, prev = it_next._next.value, it_prev._next.value
			}
			avl.remove_node(&ctx.sweep_list_, node, false)
			if next != nil && prev != nil do possibleInter(prev, next)
		}

		pop_res = PopScanline(&ctx) or_return
		if pop_res.sweep == nil do break
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
