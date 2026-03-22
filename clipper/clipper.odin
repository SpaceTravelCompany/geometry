package clipper

import "../linalg_ex"
import "base:intrinsics"
import "base:runtime"
import pq "core:container/priority_queue"
import "core:math"
import "core:slice"
import "shared:utils_private"

import "shared:utils_private/fixed_bcd"


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
Context :: struct($FRAC_DIGITS: int, $U: typeid) {
	bot_y_:              fixed_bcd.BCD(FRAC_DIGITS),
	actives_:            ^Active(FRAC_DIGITS, U),
	sel_:                ^Active(FRAC_DIGITS, U),
	minima_list_:        [dynamic]^LocalMinima(FRAC_DIGITS, U),
	vertex_lists_:       [dynamic]^Vertex(FRAC_DIGITS, U),
	scanline_list_:      pq.Priority_Queue(fixed_bcd.BCD(FRAC_DIGITS)),
	intersect_nodes_:    [dynamic]IntersectNode(FRAC_DIGITS, U),
	horz_seg_list_:      [dynamic]HorzSegment(FRAC_DIGITS, U),
	horz_join_list_:     [dynamic]HorzJoin(FRAC_DIGITS, U),
	out:                 [dynamic][dynamic]fixed_bcd.BCD(FRAC_DIGITS),
	outrec_list_:        [dynamic]^OutRec(FRAC_DIGITS, U),
	current_locmin_idx_: int,
	preserve_collinear_: bool,
	reverse_solution_:   bool,
	has_open_paths_:     bool,
	fill_rule_:          FillRule,
	clip_type_:          ClipType,
}

// Scanline: y-coordinate for sweep, linked list (clipper.engine.cpp)
@(private = "file")
Scanline :: struct($FRAC_DIGITS: int) {
	y:    fixed_bcd.BCD(FRAC_DIGITS),
	next: ^Scanline(FRAC_DIGITS),
}

// Vertex: polygon vertex, circular doubly linked (clipper.engine.h)
@(private = "file")
Vertex :: struct($FRAC_DIGITS: int, $U: typeid) {
	pt:    [2]fixed_bcd.BCD(FRAC_DIGITS),
	next:  ^Vertex(FRAC_DIGITS, U),
	prev:  ^Vertex(FRAC_DIGITS, U),
	flags: VertexFlags_Set,
	pt_u:  U,
}

// LocalMinima: bottom of an edge bound (clipper.engine.h)
@(private = "file")
LocalMinima :: struct($FRAC_DIGITS: int, $U: typeid) {
	vertex:   ^Vertex(FRAC_DIGITS, U),
	polytype: PathType,
	is_open:  bool,
}

// OutRec: path in the clipping solution; AEL edges reference it (clipper.engine.h)
@(private = "file")
OutRec :: struct($FRAC_DIGITS: int, $U: typeid) {
	idx:        int,
	owner:      ^OutRec(FRAC_DIGITS, U),
	front_edge: ^Active(FRAC_DIGITS, U),
	back_edge:  ^Active(FRAC_DIGITS, U),
	pts:        ^OutPt(FRAC_DIGITS, U),
	is_open:    bool,
}

// OutPt: output polygon vertex, circular doubly linked (clipper.engine.h)
@(private = "file")
OutPt :: struct($FRAC_DIGITS: int, $U: typeid) {
	pt:     [2]fixed_bcd.BCD(FRAC_DIGITS),
	next:   ^OutPt(FRAC_DIGITS, U),
	prev:   ^OutPt(FRAC_DIGITS, U),
	outrec: ^OutRec(FRAC_DIGITS, U),
	horz:   ^HorzSegment(FRAC_DIGITS, U),
	pt_u:   U,
}

// Active: edge in AEL/SEL (clipper.engine.h)
@(private = "file")
Active :: struct($FRAC_DIGITS: int, $U: typeid) {
	bot:           [2]fixed_bcd.BCD(FRAC_DIGITS),
	top:           [2]fixed_bcd.BCD(FRAC_DIGITS),
	curr_x:        fixed_bcd.BCD(FRAC_DIGITS),
	dx:            fixed_bcd.BCD(FRAC_DIGITS),
	outrec:        ^OutRec(FRAC_DIGITS, U),
	prev_in_ael:   ^Active(FRAC_DIGITS, U),
	next_in_ael:   ^Active(FRAC_DIGITS, U),
	prev_in_sel:   ^Active(FRAC_DIGITS, U),
	next_in_sel:   ^Active(FRAC_DIGITS, U),
	jump:          ^Active(FRAC_DIGITS, U),
	vertex_top:    ^Vertex(FRAC_DIGITS, U),
	local_min:     ^LocalMinima(FRAC_DIGITS, U),
	wind_dx:       i32, // handle negatives
	wind_cnt:      i32,
	wind_cnt2:     i32,
	bot_u:         U,
	top_u:         U,
	is_left_bound: bool,
	join_with:     JoinWith,
}

// IntersectNode: intersection point and two edges (clipper.engine.h)
@(private = "file")
IntersectNode :: struct($FRAC_DIGITS: int, $U: typeid) {
	pt:    [2]fixed_bcd.BCD(FRAC_DIGITS),
	edge1: ^Active(FRAC_DIGITS, U),
	edge2: ^Active(FRAC_DIGITS, U),
	pt_u:  U,
}

// HorzSegment: horizontal segment for join processing (clipper.engine.h)
@(private = "file")
HorzSegment :: struct($FRAC_DIGITS: int, $U: typeid) {
	left_op:       ^OutPt(FRAC_DIGITS, U),
	right_op:      ^OutPt(FRAC_DIGITS, U),
	left_to_right: bool,
}

// HorzJoin: horizontal join pair (clipper.engine.h)
@(private = "file")
HorzJoin :: struct($FRAC_DIGITS: int, $U: typeid) {
	op1: ^OutPt(FRAC_DIGITS, U),
	op2: ^OutPt(FRAC_DIGITS, U),
}

//
//입력/경로
//

@(private = "file")
AddPaths :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
	paths: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
	polytype: PathType,
	$is_open: bool,
) -> (
	err: Clipper_Error,
) {
	when is_open do ctx.has_open_paths_ = true
	AddPaths_(ctx, paths, polytype, is_open) or_return
	return
}

@(private = "file")
AddPaths_ :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
	paths: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
	polytype: PathType,
	$is_open: bool,
) -> (
	err: Clipper_Error,
) {
	total_vertices := 0
	for path in paths {
		total_vertices += len(path)
	}
	if total_vertices == 0 do return nil

	all_vertices := (^Vertex(FRAC_DIGITS, U))(
		raw_data(
			runtime.mem_alloc_non_zeroed(
				size_of(Vertex(FRAC_DIGITS, U)) * total_vertices,
				allocator = context.temp_allocator,
			) or_return,
		),
	)

	v := all_vertices
	for path in paths {
		//for each path create a circular double linked list of vertices
		v0 := v
		curr_v := v
		prev_v: ^Vertex(FRAC_DIGITS, U) = nil

		if len(path) == 0 do continue

		cnt := 0
		v.prev = nil
		for pt in path {
			if prev_v != nil {
				if prev_v.pt == pt do continue // skip duplicates
				prev_v.next = curr_v
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
			for curr_v != v0 && curr_v.pt.y == v0.pt.y {
				curr_v = curr_v.next
			}

			going_up = fixed_bcd.less_than(curr_v.pt.y, v0.pt.y)
			if going_up {
				v0.flags = {.OpenStart}
				AddLocMin(ctx, v0, polytype, true) or_return
			} else {
				v0.flags = {.OpenStart, .LocalMax}
			}

		} else { 	// closed path
			prev_v = v0.prev
			for prev_v != v0 && prev_v.pt.y == v0.pt.y {
				prev_v = prev_v.prev
			}
			if prev_v == v0 do continue // only open paths can be completely flat
			going_up = fixed_bcd.greater(prev_v.pt.y, v0.pt.y)
		}

		going_up0 = going_up
		prev_v = v0
		curr_v = v0.next
		for curr_v != v0 {
			if fixed_bcd.greater(curr_v.pt.y, prev_v.pt.y) && going_up {
				prev_v.flags += {.LocalMax}
				going_up = false
			} else if fixed_bcd.less(curr_v.pt.y, prev_v.pt.y) && !going_up {
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
	ctx: ^Context($FRAC_DIGITS, $U),
	vert: ^Vertex(FRAC_DIGITS, U),
	polytype: PathType,
	$is_open: bool,
) -> (
	err: Clipper_Error,
) {
	if .LocalMin in vert.flags do return nil //make sure the vertex is added only once ...

	vert.flags += {.LocalMin}
	lm := new(LocalMinima(FRAC_DIGITS, U), context.temp_allocator) or_return
	lm.vertex = vert
	lm.polytype = polytype
	lm.is_open = is_open
	non_zero_append(&ctx.minima_list_, lm) or_return
	return
}


@(private = "file")
AddSubject :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
	subjects: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
) -> Clipper_Error {
	if subjects == nil do return nil
	AddPaths(ctx, subjects, .Subject, false) or_return
	return nil
}

@(private = "file")
AddOpenSubject :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
	opens: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
) -> Clipper_Error {
	if opens == nil do return nil
	AddPaths(ctx, opens, .Subject, true) or_return
	return nil
}

@(private = "file")
AddClip :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
	clips: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
) -> Clipper_Error {
	if clips == nil do return nil
	AddPaths(ctx, clips, .Clip, false) or_return
	return nil
}

//
//스캔라인/로컬미니마
//

@(private = "file")
PopScanline :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
	inout_y: ^fixed_bcd.BCD(FRAC_DIGITS),
) -> (
	res: bool,
	err: Clipper_Error,
) {
	if pq.len(ctx.scanline_list_) == 0 {
		return false, nil
	}

	inout_y^ = pq.pop(&ctx.scanline_list_)

	for pq.len(ctx.scanline_list_) != 0 && (inout_y^ == pq.peek(ctx.scanline_list_)) {
		pq.pop(&ctx.scanline_list_)
	}

	return true, nil
}

//
//Active 판별
//

@(private = "file")
GetPolyType :: proc "contextless" (e: ^Active($FRAC_DIGITS, $U)) -> PathType {
	return e.local_min.polytype
}

@(private = "file")
PopLocalMinima :: proc "contextless" (
	ctx: ^Context($FRAC_DIGITS, $U),
	y: fixed_bcd.BCD(FRAC_DIGITS),
) -> ^LocalMinima(FRAC_DIGITS, U) {
	if ctx.current_locmin_idx_ >= len(ctx.minima_list_) ||
	   !fixed_bcd.equal(y, ctx.minima_list_[ctx.current_locmin_idx_].vertex.pt.y) {
		return nil
	}

	res := ctx.minima_list_[ctx.current_locmin_idx_]
	ctx.current_locmin_idx_ += 1
	return res
}

@(private = "file")
IsOpenActive :: proc "contextless" (e: ^Active($FRAC_DIGITS, $U)) -> bool {
	return e.local_min.is_open
}

@(private = "file")
IsOpenEndVertex :: proc "contextless" (v: ^Vertex($FRAC_DIGITS, $U)) -> bool {
	return .OpenEnd in v.flags || .OpenStart in v.flags
}

@(private = "file")
IsHorizontal :: proc "contextless" (e: ^Active($FRAC_DIGITS, $U)) -> bool {
	return fixed_bcd.equal(e.top.y, e.bot.y)
}

@(private = "file")
GetDx :: proc "contextless" (
	pt1, pt2: [2]fixed_bcd.BCD($FRAC_DIGITS),
) -> fixed_bcd.BCD(FRAC_DIGITS) {
	dy := fixed_bcd.sub(pt2.y, pt1.y)
	context = runtime.default_context()
	if dy.i != 0 {
		return fixed_bcd.div(fixed_bcd.sub(pt2.x, pt1.x), dy)
	}
	if pt2.x.i > pt1.x.i do return fixed_bcd.inf_min(FRAC_DIGITS)
	return fixed_bcd.inf_max(FRAC_DIGITS)
}

@(private = "file")
GetLineIntersectPt :: proc "contextless" (
	a1: [2]fixed_bcd.BCD($FRAC_DIGITS),
	a2: [2]fixed_bcd.BCD(FRAC_DIGITS),
	b1: [2]fixed_bcd.BCD(FRAC_DIGITS),
	b2: [2]fixed_bcd.BCD(FRAC_DIGITS),
) -> (
	ok: bool,
	ip: [2]fixed_bcd.BCD(FRAC_DIGITS),
) {
	den := fixed_bcd.sub(
		fixed_bcd.mul(fixed_bcd.sub(b2.y, b1.y), fixed_bcd.sub(a2.x, a1.x)),
		fixed_bcd.mul(fixed_bcd.sub(b2.x, b1.x), fixed_bcd.sub(a2.y, a1.y)),
	)
	if den.i == 0 do return false, {}

	ua := fixed_bcd.sub(
		fixed_bcd.mul(fixed_bcd.sub(b2.x, b1.x), fixed_bcd.sub(a1.y, b1.y)),
		fixed_bcd.mul(fixed_bcd.sub(b2.y, b1.y), fixed_bcd.sub(a1.x, b1.x)),
	)
	t := fixed_bcd.div(ua, den)
	ip = [2]fixed_bcd.BCD(FRAC_DIGITS) {
		fixed_bcd.add(a1.x, fixed_bcd.mul(t, fixed_bcd.sub(a2.x, a1.x))),
		fixed_bcd.add(a1.y, fixed_bcd.mul(t, fixed_bcd.sub(a2.y, a1.y))),
	}
	return true, ip
}

@(private = "file")
SetDx :: proc "contextless" (e: ^Active($FRAC_DIGITS, $U)) {
	e.dx = GetDx(e.bot, e.top)
}

@(private = "file")
IsHeadingLeftHorz :: proc "contextless" (e: ^Active($FRAC_DIGITS, $U)) -> bool {
	return e.dx == fixed_bcd.inf_max(FRAC_DIGITS)
}

@(private = "file")
IsHeadingRightHorz :: proc "contextless" (e: ^Active($FRAC_DIGITS, $U)) -> bool {
	return e.dx == fixed_bcd.inf_min(FRAC_DIGITS)
}

@(private = "file")
IsContributingClosed :: proc "contextless" (
	ctx: ^Context($FRAC_DIGITS, $U),
	e: ^Active(FRAC_DIGITS, U),
) -> bool {
	#partial switch ctx.fill_rule_ {
	case .NonZero:
		if abs(e.wind_cnt) != 1 do return false
	case .Positive:
		if e.wind_cnt != 1 do return false
	case .Negative:
		if e.wind_cnt != -1 do return false
	}

	#partial switch ctx.clip_type_ {
	case .Intersection:
		#partial switch ctx.fill_rule_ {
		case .Positive:
			return e.wind_cnt2 > 0
		case .Negative:
			return e.wind_cnt2 < 0
		case:
			return e.wind_cnt2 != 0
		}

	case .Union:
		#partial switch ctx.fill_rule_ {
		case .Positive:
			return e.wind_cnt2 <= 0
		case .Negative:
			return e.wind_cnt2 >= 0
		case:
			return e.wind_cnt2 == 0
		}

	case .Difference:
		result: bool
		#partial switch ctx.fill_rule_ {
		case .Positive:
			result = e.wind_cnt2 <= 0
		case .Negative:
			result = e.wind_cnt2 >= 0
		case:
			result = e.wind_cnt2 == 0
		}

		if GetPolyType(e) == .Subject do return result
		else do return !result
	case .Xor:
		return true
	}
	return false // we should never get here
}

@(private = "file")
IsContributingOpen :: proc "contextless" (
	ctx: ^Context($FRAC_DIGITS, $U),
	e: ^Active(FRAC_DIGITS, U),
) -> bool {
	is_in_clip: bool
	is_in_subj: bool
	#partial switch ctx.fill_rule_ {
	case .Positive:
		is_in_clip = e.wind_cnt2 > 0
		is_in_subj = e.wind_cnt > 0
	case .Negative:
		is_in_clip = e.wind_cnt2 < 0
		is_in_subj = e.wind_cnt < 0
	case:
		is_in_clip = e.wind_cnt2 != 0
		is_in_subj = e.wind_cnt != 0
	}

	#partial switch ctx.clip_type_ {
	case .Intersection:
		return is_in_clip
	case .Union:
		return !is_in_subj && !is_in_clip
	case:
		return !is_in_clip
	}
}

//
//AEL 삽입/윈드
//

@(private = "file")
TopX :: proc "contextless" (
	ae: ^Active($FRAC_DIGITS, $U),
	current_y: fixed_bcd.BCD(FRAC_DIGITS),
) -> fixed_bcd.BCD(FRAC_DIGITS) {
	if fixed_bcd.equal(current_y, ae.top.y) || fixed_bcd.equal(ae.top.x, ae.bot.x) do return ae.top.x
	if fixed_bcd.equal(current_y, ae.bot.y) do return ae.bot.x

	return fixed_bcd.add(ae.bot.x, fixed_bcd.mul(ae.dx, fixed_bcd.sub(current_y, ae.bot.y)))
}

@(private = "file")
TrimHorz :: proc "contextless" (horz_edge: ^Active($FRAC_DIGITS, $U), preserve_collinear: bool) {
	was_trimmed := false
	pt := NextVertex(horz_edge).pt

	for fixed_bcd.equal(pt.y, horz_edge.top.y) {
		// always trim 180 deg. spikes (in closed paths)
		// but otherwise break if preserveCollinear = true
		if preserve_collinear &&
		   (fixed_bcd.less(pt.x, horz_edge.top.x) !=
				   fixed_bcd.less(horz_edge.bot.x, horz_edge.top.x)) {
			break
		}

		horz_edge.vertex_top = NextVertex(horz_edge)
		horz_edge.top = pt
		was_trimmed = true

		if IsMaxima(horz_edge.vertex_top) do break

		pt = NextVertex(horz_edge).pt
	}
	if was_trimmed do SetDx(horz_edge) // +/-infinity
}

@(private = "file")
UpdateEdgeIntoAEL :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
	e: ^Active(FRAC_DIGITS, U),
) -> Clipper_Error {
	e.bot = e.top
	e.vertex_top = NextVertex(e)
	e.top = e.vertex_top.pt
	e.curr_x = e.bot.x
	SetDx(e)

	if IsJoined(e) do Split(ctx, e, e.bot) or_return

	if IsHorizontal(e) {
		if !IsOpenActive(e) do TrimHorz(e, ctx.preserve_collinear_)
		return nil
	}

	InsertScanline(ctx, e.top.y) or_return
	CheckJoinLeft(ctx, e, e.bot) or_return
	CheckJoinRight(ctx, e, e.bot, true) or_return
	return nil
}

@(private = "file")
DeleteFromAEL :: proc "contextless" (ctx: ^Context($FRAC_DIGITS, $U), e: ^Active(FRAC_DIGITS, U)) {
	prev := e.prev_in_ael
	next := e.next_in_ael

	if prev == nil && next == nil && e != ctx.actives_ do return

	if prev != nil do prev.next_in_ael = next
	else do ctx.actives_ = next

	if next != nil do next.prev_in_ael = prev
	// delete e
}

@(private = "file")
AdjustCurrXAndCopyToSEL :: proc "contextless" (
	ctx: ^Context($FRAC_DIGITS, $U),
	top_y: fixed_bcd.BCD(FRAC_DIGITS),
) {
	e := ctx.actives_
	ctx.sel_ = e

	for e != nil {
		e.prev_in_sel = e.prev_in_ael
		e.next_in_sel = e.next_in_ael

		e.jump = e.next_in_sel
		// it is safe to ignore 'joined' edges here because
		// if necessary they will be split in IntersectEdges()
		e.curr_x = TopX(e, top_y)
		e = e.next_in_ael
	}
}

@(private = "file")
InsertLocalMinimaIntoAEL :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
	bot_y: fixed_bcd.BCD(FRAC_DIGITS),
) -> Clipper_Error {
	left_bound: ^Active(FRAC_DIGITS, U)
	right_bound: ^Active(FRAC_DIGITS, U)
	//Add any local minima (if any) at BotY ...
	//nb: horizontal local minima edges should contain locMin.vertex.prev

	for {
		local_minima := PopLocalMinima(ctx, bot_y)
		if local_minima == nil do break

		if .OpenStart in local_minima.vertex.flags {
			left_bound = nil
		} else {
			left_bound = new_clone(
				Active(FRAC_DIGITS, U) {
					bot        = local_minima.vertex.pt,
					curr_x     = local_minima.vertex.pt.x,
					wind_dx    = -1,
					vertex_top = local_minima.vertex.prev, // descending
					top        = local_minima.vertex.prev.pt,
					local_min  = local_minima,
				},
				context.temp_allocator,
			) or_return
			SetDx(left_bound)
		}

		if .OpenEnd in local_minima.vertex.flags {
			right_bound = nil
		} else {
			right_bound = new_clone(
				Active(FRAC_DIGITS, U) {
					bot        = local_minima.vertex.pt,
					curr_x     = local_minima.vertex.pt.x,
					wind_dx    = 1,
					vertex_top = local_minima.vertex.next, // ascending
					top        = local_minima.vertex.next.pt,
					local_min  = local_minima,
				},
				context.temp_allocator,
			) or_return
			SetDx(right_bound)
		}

		//Currently LeftB is just the descending bound and RightB is the ascending.
		//Now if the LeftB isn't on the left of RightB then we need swap them.
		if left_bound != nil && right_bound != nil {
			if IsHorizontal(left_bound) {
				if IsHeadingRightHorz(left_bound) do left_bound, right_bound = right_bound, left_bound
			} else if IsHorizontal(right_bound) {
				if IsHeadingLeftHorz(right_bound) do left_bound, right_bound = right_bound, left_bound
			} else if left_bound.dx.i < right_bound.dx.i do left_bound, right_bound = right_bound, left_bound

		} else if left_bound == nil {
			left_bound = right_bound
			right_bound = nil
		}

		contributing: bool
		left_bound.is_left_bound = true
		InsertLeftEdge(ctx, left_bound)

		if IsOpenActive(left_bound) {
			SetWindCountForOpenPathEdge(ctx, left_bound)
			contributing = IsContributingOpen(ctx, left_bound)
		} else {
			SetWindCountForClosedPathEdge(ctx, left_bound)
			contributing = IsContributingClosed(ctx, left_bound)
		}

		if right_bound != nil {
			right_bound.is_left_bound = false
			right_bound.wind_cnt = left_bound.wind_cnt
			right_bound.wind_cnt2 = left_bound.wind_cnt2
			InsertRightEdge(left_bound, right_bound)

			if contributing {
				AddLocalMinPoly(ctx, left_bound, right_bound, left_bound.bot, true) or_return
				if !IsHorizontal(left_bound) do CheckJoinLeft(ctx, left_bound, left_bound.bot) or_return
			}

			for right_bound.next_in_ael != nil &&
			    IsValidAelOrder(right_bound.next_in_ael, right_bound) {

				IntersectEdges(
					ctx,
					right_bound,
					right_bound.next_in_ael,
					right_bound.bot,
				) or_return
				SwapPositionsInAEL(ctx, right_bound, right_bound.next_in_ael)
			}

			if IsHorizontal(right_bound) {
				PushHorz(ctx, right_bound)
			} else {
				CheckJoinRight(ctx, right_bound, right_bound.bot) or_return
				InsertScanline(ctx, right_bound.top.y) or_return
			}
		} else if contributing {
			StartOpenPath(ctx, left_bound, left_bound.bot) or_return
		}

		if IsHorizontal(left_bound) {
			PushHorz(ctx, left_bound)
		} else {
			InsertScanline(ctx, left_bound.top.y) or_return
		}
	}
	return nil
}

@(private = "file")
InsertLeftEdge :: proc "contextless" (
	ctx: ^Context($FRAC_DIGITS, $U),
	e: ^Active(FRAC_DIGITS, U),
) {
	if ctx.actives_ == nil {
		e.prev_in_ael = nil
		e.next_in_ael = nil
		ctx.actives_ = e

	} else if !IsValidAelOrder(ctx.actives_, e) {
		e.prev_in_ael = nil
		e.next_in_ael = ctx.actives_
		ctx.actives_.prev_in_ael = e
		ctx.actives_ = e

	} else {
		e2 := ctx.actives_
		for e2.next_in_ael != nil && IsValidAelOrder(e2.next_in_ael, e) {
			e2 = e2.next_in_ael
		}

		if e2.join_with == .Right {
			e2 = e2.next_in_ael
		}
		//if e2 == nil do return // should never happen and stops compiler warning (original code)

		e.next_in_ael = e2.next_in_ael
		if e2.next_in_ael != nil do e2.next_in_ael.prev_in_ael = e
		e.prev_in_ael = e2
		e2.next_in_ael = e
	}
}

@(private = "file")
InsertRightEdge :: proc "contextless" (e: ^Active($FRAC_DIGITS, $U), e2: ^Active(FRAC_DIGITS, U)) {
	e2.next_in_ael = e.next_in_ael
	if e.next_in_ael != nil do e.next_in_ael.prev_in_ael = e2
	e2.prev_in_ael = e
	e.next_in_ael = e2
}

@(private = "file")
SetWindCountForOpenPathEdge :: proc "contextless" (
	ctx: ^Context($FRAC_DIGITS, $U),
	e: ^Active(FRAC_DIGITS, U),
) {
	e2 := ctx.actives_
	if ctx.fill_rule_ == .EvenOdd {
		cnt1, cnt2 := 0, 0
		for e2 != e {
			if GetPolyType(e2) == .Clip do cnt2 += 1
			else if !IsOpenActive(e2) do cnt1 += 1

			e2 = e2.next_in_ael
		}

		e.wind_cnt = (cnt1 % 2) != 0 ? 1 : 0
		e.wind_cnt2 = (cnt2 % 2) != 0 ? 1 : 0
	} else {
		for e2 != e {
			if GetPolyType(e2) == .Clip do e.wind_cnt2 += e2.wind_dx
			else if !IsOpenActive(e2) do e.wind_cnt += e2.wind_dx

			e2 = e2.next_in_ael
		}
	}
}

@(private = "file")
SetWindCountForClosedPathEdge :: proc "contextless" (
	ctx: ^Context($FRAC_DIGITS, $U),
	e: ^Active(FRAC_DIGITS, U),
) {
	//Wind counts refer to polygon regions not edges, so here an edge's WindCnt
	//indicates the higher of the wind counts for the two regions touching the edge.
	//(NB Adjacent regions can only ever have their wind counts differ by one.
	//Also, open paths have no meaningful wind directions or counts.)

	e2 := e.prev_in_ael
	//find the nearest closed path edge of the same PolyType in AEL (heading left)
	pt := GetPolyType(e)
	for e2 != nil && (GetPolyType(e2) != pt || IsOpenActive(e2)) do e2 = e2.prev_in_ael

	if e2 == nil {
		e.wind_cnt = e.wind_dx
		e2 = ctx.actives_

	} else if ctx.fill_rule_ == .EvenOdd {
		e.wind_cnt = e.wind_dx
		e.wind_cnt2 = e2.wind_cnt2
		e2 = e2.next_in_ael

	} else {
		// NonZero, positive, or negative filling
		// If e's WindCnt has the SAME sign as WindDx, filling is on the right of e.
		//NB neither e2.WindCnt nor e2.WindDx should ever be 0.
		if e2.wind_cnt * e2.wind_dx < 0 {
			// opposite directions so e is outside e2
			if abs(e2.wind_cnt) > 1 {
				if e2.wind_dx * e.wind_dx < 0 do e.wind_cnt = e2.wind_cnt
				else do e.wind_cnt = e2.wind_cnt + e.wind_dx

			} else {
				//now outside all polys of same polytype so set own WC ...
				e.wind_cnt = IsOpenActive(e) ? 1 : e.wind_dx
			}
		} else {
			//'e' must be inside 'e2'
			// reversing direction so use the same WC
			if e2.wind_dx * e.wind_dx < 0 do e.wind_cnt = e2.wind_cnt
			else do e.wind_cnt = e2.wind_cnt + e.wind_dx //otherwise keep 'increasing' the WC by 1 (away from 0) ...	
		}
		e.wind_cnt2 = e2.wind_cnt2
		e2 = e2.next_in_ael // get ready to calc WindCnt2
	}
	// update wind_cnt2
	if ctx.fill_rule_ == .EvenOdd {
		for e2 != e {
			if GetPolyType(e2) != pt && !IsOpenActive(e2) do e.wind_cnt2 = e.wind_cnt2 == 0 ? 1 : 0
			e2 = e2.next_in_ael
		}
	} else {
		for e2 != e {
			if GetPolyType(e2) != pt && !IsOpenActive(e2) do e.wind_cnt2 += e2.wind_dx
			e2 = e2.next_in_ael
		}
	}
}

@(private = "file")
CheckJoinLeft :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
	e: ^Active(FRAC_DIGITS, U),
	pt: [2]fixed_bcd.BCD(FRAC_DIGITS),
	check_curr_x: bool = false,
) -> Clipper_Error {
	prev := e.prev_in_ael

	if prev == nil ||
	   !IsHotEdge(e) ||
	   !IsHotEdge(prev) ||
	   IsHorizontal(e) ||
	   IsHorizontal(prev) ||
	   IsOpenActive(e) ||
	   IsOpenActive(prev) {
		return nil
	}

	if ((pt.y.i <= e.top.y.i + 1) || (pt.y.i <= prev.top.y.i + 1)) &&
	   (fixed_bcd.greater(e.bot.y, pt.y) || fixed_bcd.greater(prev.bot.y, pt.y)) {
		return nil // avoid trivial joins
	}

	if check_curr_x {
		a, b := PerpendicDistFromLineSqrd(pt, prev.bot, prev.top)
		if a.i > 1 do return nil // b > 0 always
	} else if e.curr_x != prev.curr_x {
		return nil
	}
	if !IsCollinear(e.top, pt, prev.top) do return nil

	if e.outrec.idx == prev.outrec.idx {
		AddLocalMaxPoly(ctx, prev, e, pt) or_return
	} else if e.outrec.idx < prev.outrec.idx {
		JoinOutrecPaths(ctx, e, prev)
	} else {
		JoinOutrecPaths(ctx, prev, e)
	}

	prev.join_with = .Right
	e.join_with = .Left
	return nil
}

@(private = "file")
CheckJoinRight :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
	e: ^Active(FRAC_DIGITS, U),
	pt: [2]fixed_bcd.BCD(FRAC_DIGITS),
	check_curr_x: bool = false,
) -> Clipper_Error {
	next := e.next_in_ael
	if next == nil ||
	   !IsHotEdge(e) ||
	   !IsHotEdge(next) ||
	   IsHorizontal(e) ||
	   IsHorizontal(next) ||
	   IsOpenActive(e) ||
	   IsOpenActive(next) {
		return nil
	}

	trivial_y := pt.y.i <= e.top.y.i + 1 || pt.y.i <= next.top.y.i + 1
	trivial_bot := fixed_bcd.greater(e.bot.y, pt.y) || fixed_bcd.greater(next.bot.y, pt.y)
	if trivial_y && trivial_bot do return nil // avoid trivial joins

	if check_curr_x {
		a, b := PerpendicDistFromLineSqrd(pt, next.bot, next.top)

		if a.i > 1 do return nil
	} else if e.curr_x != next.curr_x {
		return nil
	}
	if !IsCollinear(e.top, pt, next.top) do return nil

	if e.outrec.idx == next.outrec.idx {
		AddLocalMaxPoly(ctx, e, next, pt) or_return
	} else if e.outrec.idx < next.outrec.idx {
		JoinOutrecPaths(ctx, e, next)
	} else {
		JoinOutrecPaths(ctx, next, e)
	}
	e.join_with = .Right
	next.join_with = .Left
	return nil
}

//
//OutRec/소유
//

@(private = "file")
OutrecIsAscending :: proc "contextless" (hot_edge: ^Active($FRAC_DIGITS, $U)) -> bool {
	return hot_edge == hot_edge.outrec.front_edge
}

@(private = "file")
IsOpenEnd :: proc "contextless" (ae: ^Active($FRAC_DIGITS, $U)) -> bool {
	return IsOpenEndVertex(ae.vertex_top)
}

@(private = "file")
GetPrevHotEdge :: proc "contextless" (
	ctx: ^Context($FRAC_DIGITS, $U),
	e: ^Active(FRAC_DIGITS, U),
) -> ^Active(FRAC_DIGITS, U) {
	prev := e.prev_in_ael
	for prev != nil && (IsOpenActive(prev) || !IsHotEdge(prev)) {
		prev = prev.prev_in_ael
	}
	return prev
}

@(private = "file")
SetOwner :: proc "contextless" (
	outrec: ^OutRec($FRAC_DIGITS, $U),
	new_owner: ^OutRec(FRAC_DIGITS, U),
) {
	// precondition: new_owner is never null
	new_owner.owner = GetRealOutRec(new_owner.owner)
	tmp := new_owner
	for tmp != nil && tmp != outrec {
		tmp = tmp.owner
	}
	if tmp != nil do new_owner.owner = outrec.owner
	outrec.owner = new_owner
}

@(private = "file")
SwapFrontBackSides :: proc "contextless" (outrec: ^OutRec($FRAC_DIGITS, $U)) {
	tmp := outrec.front_edge
	outrec.front_edge = outrec.back_edge
	outrec.back_edge = tmp
	outrec.pts = outrec.pts.next
}

@(private = "file")
UncoupleOutRec :: proc "contextless" (e: ^Active($FRAC_DIGITS, $U)) {
	outrec := e.outrec
	if outrec == nil do return
	outrec.front_edge.outrec = nil
	outrec.back_edge.outrec = nil
	outrec.front_edge = nil
	outrec.back_edge = nil
}

//
//로컬 min/max/join
//

@(private = "file")
AddLocalMinPoly :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
	e1: ^Active(FRAC_DIGITS, U),
	e2: ^Active(FRAC_DIGITS, U),
	pt: [2]fixed_bcd.BCD(FRAC_DIGITS),
	is_new: bool = false,
) -> (
	res: ^OutPt(FRAC_DIGITS, U),
	err: Clipper_Error,
) {
	outrec := NewOutRec(ctx) or_return
	e1.outrec = outrec
	e2.outrec = outrec

	if IsOpenActive(e1) {
		outrec.owner = nil
		outrec.is_open = true
		if e1.wind_dx > 0 do SetSides(outrec, e1, e2)
		else do SetSides(outrec, e2, e1)

	} else {
		prev_hot_edge := GetPrevHotEdge(ctx, e1)
		// e.windDx is the winding direction of the **input** paths
		// and unrelated to the winding direction of output polygons.
		// Output orientation is determined by e.outrec.frontE which is
		// the ascending edge (see AddLocalMinPoly).
		if prev_hot_edge != nil {
			if OutrecIsAscending(prev_hot_edge) == is_new {
				SetSides(outrec, e2, e1)
			} else {
				SetSides(outrec, e1, e2)
			}

		} else {
			outrec.owner = nil
			if is_new {
				SetSides(outrec, e1, e2)
			} else {
				SetSides(outrec, e2, e1)
			}
		}
	}

	res = new_clone(
		OutPt(FRAC_DIGITS, U){pt = pt, outrec = outrec},
		context.temp_allocator,
	) or_return
	res.next = res
	res.prev = res
	outrec.pts = res
	return
}

@(private = "file")
AddLocalMaxPoly :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
	e1: ^Active(FRAC_DIGITS, U),
	e2: ^Active(FRAC_DIGITS, U),
	pt: [2]fixed_bcd.BCD(FRAC_DIGITS),
) -> (
	res: ^OutPt(FRAC_DIGITS, U),
	err: Clipper_Error,
) {
	if IsJoined(e1) do Split(ctx, e1, pt) or_return
	if IsJoined(e2) do Split(ctx, e2, pt) or_return

	if IsFront(e1) == IsFront(e2) {
		if IsOpenEnd(e1) {
			SwapFrontBackSides(e1.outrec)
		} else if IsOpenEnd(e2) {
			SwapFrontBackSides(e2.outrec)
		} else {
			return nil, .ADD_LOCAL_MAX_POLY_FAILED
		}
	}

	result := AddOutPt(ctx, e1, pt) or_return
	if e1.outrec == e2.outrec {
		outrec := e1.outrec
		outrec.pts = result

		UncoupleOutRec(e1)
		result = outrec.pts

		if outrec.owner != nil && outrec.owner.front_edge == nil {
			outrec.owner = GetRealOutRec(outrec.owner)
		}
	} else if IsOpenActive(e1) { 	//and to preserve the winding orientation of outrec ...
		if e1.wind_dx < 0 {
			JoinOutrecPaths(ctx, e1, e2)
		} else {
			JoinOutrecPaths(ctx, e2, e1)
		}

	} else if e1.outrec.idx < e2.outrec.idx {
		JoinOutrecPaths(ctx, e1, e2)
	} else {
		JoinOutrecPaths(ctx, e2, e1)
	}
	return result, nil
}


@(private = "file")
JoinOutrecPaths :: proc "contextless" (
	ctx: ^Context($FRAC_DIGITS, $U),
	e1: ^Active(FRAC_DIGITS, U),
	e2: ^Active(FRAC_DIGITS, U),
) {
	// Join e2 outrec path onto e1 outrec path, then clear e2 outrec path.
	// (Joining ends rarely share the same coords.)
	p1_st := e1.outrec.pts
	p2_st := e2.outrec.pts
	p1_end := p1_st.next
	p2_end := p2_st.next

	if IsFront(e1) {
		p2_end.prev = p1_st
		p1_st.next = p2_end
		p2_st.next = p1_end
		p1_end.prev = p2_st
		e1.outrec.pts = p2_st
		e1.outrec.front_edge = e2.outrec.front_edge

		if e1.outrec.front_edge != nil {
			e1.outrec.front_edge.outrec = e1.outrec
		}
	} else {
		p1_end.prev = p2_st
		p2_st.next = p1_end
		p1_st.next = p2_end
		p2_end.prev = p1_st
		e1.outrec.back_edge = e2.outrec.back_edge

		if e1.outrec.back_edge != nil {
			e1.outrec.back_edge.outrec = e1.outrec
		}
	}
	// After joining, e2.outrec must hold no vertices
	e2.outrec.front_edge = nil
	e2.outrec.back_edge = nil
	e2.outrec.pts = nil

	if IsOpenEnd(e1) {
		e2.outrec.pts = e1.outrec.pts
		e1.outrec.pts = nil
	} else {
		SetOwner(e2.outrec, e1.outrec)
	}
	// e1 and e2 are maxima and about to be dropped from the Actives list
	e1.outrec = nil
	e2.outrec = nil
}

//
//OutPt/OutRec/경로
//

@(private = "file")
NewOutRec :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
) -> (
	res: ^OutRec(FRAC_DIGITS, U),
	err: Clipper_Error,
) {
	res = new_clone(
		OutRec(FRAC_DIGITS, U){idx = len(ctx.outrec_list_)},
		context.temp_allocator,
	) or_return

	non_zero_append(&ctx.outrec_list_, res) or_return
	return
}

@(private = "file")
AddOutPt :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
	e: ^Active(FRAC_DIGITS, U),
	pt: [2]fixed_bcd.BCD(FRAC_DIGITS),
) -> (
	res: ^OutPt(FRAC_DIGITS, U),
	err: Clipper_Error,
) {
	// Outrec.OutPts: a circular doubly-linked-list of POutPt where ...
	// op_front[.Prev]* ~~~> op_back & op_back == op_front.Next
	outrec := e.outrec
	to_front := IsFront(e)
	op_front := outrec.pts
	op_back := op_front.next

	if to_front {
		if pt == op_front.pt do return op_front, nil
	} else if pt == op_back.pt {
		return op_back, nil
	}

	res = new_clone(
		OutPt(FRAC_DIGITS, U){pt = pt, outrec = outrec, prev = op_front, next = op_back},
		context.temp_allocator,
	) or_return

	op_back.prev = res
	op_front.next = res

	if to_front do outrec.pts = res
	return
}

@(private = "file")
GetRealOutRec :: proc "contextless" (
	outrec: ^OutRec($FRAC_DIGITS, $U),
) -> ^OutRec(FRAC_DIGITS, U) {
	outrec := outrec
	for outrec != nil && outrec.pts == nil do outrec = outrec.owner
	return outrec
}

@(private = "file")
IsValidClosedPath :: proc "contextless" (op: ^OutPt($FRAC_DIGITS, $U)) -> bool {
	return op != nil && (op.next != op) && (op.next != op.prev)
}

@(private = "file")
DisposeOutPt :: proc "contextless" (op: ^OutPt($FRAC_DIGITS, $U)) -> ^OutPt(FRAC_DIGITS, U) {
	result := op.next
	op.prev.next = op.next
	op.next.prev = op.prev
	return result
}

@(private = "file")
DisposeOutPts :: proc "contextless" (
	ctx: ^Context($FRAC_DIGITS, $U),
	outrec: ^OutRec(FRAC_DIGITS, U),
) {
	outrec.pts.prev.next = nil
	outrec.pts = nil
}

//
//Split/자기교차
//

@(private = "file")
Split :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
	e: ^Active(FRAC_DIGITS, U),
	pt: [2]fixed_bcd.BCD(FRAC_DIGITS),
) -> (
	err: Clipper_Error,
) {
	if e.join_with == .Right {
		e.join_with = .NoJoin
		e.next_in_ael.join_with = .NoJoin
		AddLocalMinPoly(ctx, e, e.next_in_ael, pt, true) or_return
	} else {
		e.join_with = .NoJoin
		e.prev_in_ael.join_with = .NoJoin
		AddLocalMinPoly(ctx, e.prev_in_ael, e, pt, true) or_return
	}
	return
}

@(private = "file")
DoSplitOp :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
	outrec: ^OutRec(FRAC_DIGITS, U),
	split_op: ^OutPt(FRAC_DIGITS, U),
) -> (
	err: Clipper_Error,
) {
	// splitOp.prev -> splitOp &&
	// splitOp.next -> splitOp.next.next are intersecting
	prev_op := split_op.prev
	next_next_op := split_op.next.next
	outrec.pts = prev_op
	_, ip := GetLineIntersectPt(prev_op.pt, split_op.pt, split_op.next.pt, next_next_op.pt)


	double_area1 := Area_Double(outrec.pts)
	double_abs_area1 := double_area1 // always positive
	if double_abs_area1.i < 0 do double_abs_area1.i = -double_abs_area1.i

	if fixed_bcd.less(double_abs_area1, fixed_bcd.init_const(2 * 2, 0, 0, FRAC_DIGITS)) {
		DisposeOutPts(ctx, outrec)
		return
	}

	area2 := AreaTriangle(ip, split_op.pt, split_op.next.pt) // always positive
	abs_area2 := area2

	// de-link splitOp and splitOp.next from the path
	// while inserting the intersection point
	if fixed_bcd.equal(ip, prev_op.pt) || fixed_bcd.equal(ip, next_next_op.pt) {
		next_next_op.prev = prev_op
		prev_op.next = next_next_op

	} else {
		new_op2 := new_clone(
			OutPt(FRAC_DIGITS, U) {
				pt = ip,
				outrec = prev_op.outrec,
				prev = prev_op,
				next = next_next_op,
			},
			context.temp_allocator,
		) or_return
		next_next_op.prev = new_op2
		prev_op.next = new_op2
	}

	// area1 is the path's area *before* splitting, whereas area2 is
	// the area of the triangle containing splitOp & splitOp.next.
	// So the only way for these areas to have the same sign is if
	// the split triangle is larger than the path containing prevOp or
	// if there's more than one self-intersection.
	if !fixed_bcd.less(abs_area2, fixed_bcd.init_const(1, 0, 0, FRAC_DIGITS)) &&
	   (fixed_bcd.greater(
				   fixed_bcd.mul(abs_area2, fixed_bcd.init_const(2, 0, 0, FRAC_DIGITS)),
				   double_abs_area1,
			   ) ||
			   (area2.i > 0) == (double_area1.i > 0)) {
		newOr := NewOutRec(ctx) or_return
		newOr.owner = outrec.owner

		split_op.outrec = newOr
		split_op.next.outrec = newOr
		new_op := new_clone(
			OutPt(FRAC_DIGITS, U){pt = ip, outrec = newOr, prev = split_op.next, next = split_op},
			context.temp_allocator,
		) or_return

		newOr.pts = new_op
		split_op.prev = new_op
		split_op.next.next = new_op
	} else {
		// _ = DisposeOutPt(split_op.next)
		// _ = DisposeOutPt(split_op)
	}
	return
}

@(private = "file")
FixSelfIntersects :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
	outrec: ^OutRec(FRAC_DIGITS, U),
) -> (
	err: Clipper_Error,
) {
	op2 := outrec.pts
	if op2.prev == op2.next.next do return // because triangles can't self-intersect

	for {
		if SegmentsIntersectExclusive(op2.prev.pt, op2.pt, op2.next.pt, op2.next.next.pt) {
			if op2 == outrec.pts || op2.next == outrec.pts do outrec.pts = outrec.pts.prev

			DoSplitOp(ctx, outrec, op2) or_return

			if outrec.pts == nil do break

			op2 = outrec.pts

			if op2.prev == op2.next.next do break // again, because triangles can't self-intersect
			continue
		} else {
			op2 = op2.next
		}
		if op2 == outrec.pts do break
	}
	return
}

@(private = "file")
UpdateOutrecOwner :: proc "contextless" (outrec: ^OutRec($FRAC_DIGITS, $U)) {
	op_curr := outrec.pts
	for {
		op_curr.outrec = outrec
		op_curr = op_curr.next
		if op_curr == outrec.pts do return
	}
}

//
//기타 private
//

@(private = "file")
NextVertex :: proc "contextless" (e: ^Active($FRAC_DIGITS, $U)) -> ^Vertex(FRAC_DIGITS, U) {
	if e.wind_dx > 0 do return e.vertex_top.next
	return e.vertex_top.prev
}

@(private = "file")
IsMaxima :: proc "contextless" (e: ^Vertex($FRAC_DIGITS, $U)) -> bool {
	return .LocalMax in e.flags
}

//PrevPrevVertex: useful to get the (inverted Y-axis) top of the
//alternate edge (ie left or right bound) during edge insertion.
@(private = "file")
PrevPrevVertex :: proc "contextless" (ae: ^Active($FRAC_DIGITS, $U)) -> ^Vertex(FRAC_DIGITS, U) {
	if ae.wind_dx > 0 do return ae.vertex_top.prev.prev
	return ae.vertex_top.next.next
}

@(private = "file")
GetMaximaPair :: proc "contextless" (e: ^Active($FRAC_DIGITS, $U)) -> ^Active(FRAC_DIGITS, U) {
	e2 := e.next_in_ael
	for e2 != nil {
		if e2.vertex_top == e.vertex_top do return e2
		e2 = e2.next_in_ael
	}
	return nil
}

@(private = "file")
IsJoined :: proc "contextless" (e: ^Active($FRAC_DIGITS, $U)) -> bool {
	return e.join_with != .NoJoin
}

@(private = "file")
IsHotEdge :: proc "contextless" (e: ^Active($FRAC_DIGITS, $U)) -> bool {
	return e.outrec != nil
}

@(private = "file")
IsFront :: proc "contextless" (e: ^Active($FRAC_DIGITS, $U)) -> bool {
	return e == e.outrec.front_edge
}

@(private = "file")
IsInvalidPath :: proc "contextless" (op: ^OutPt($FRAC_DIGITS, $U)) -> bool {
	return op == nil || op.next == op
}

@(private = "file")
SetSides :: proc "contextless" (
	outrec: ^OutRec($FRAC_DIGITS, $U),
	start_edge: ^Active(FRAC_DIGITS, U),
	end_edge: ^Active(FRAC_DIGITS, U),
) {
	outrec.front_edge = start_edge
	outrec.back_edge = end_edge
}

@(private = "file")
IsCollinear :: proc "contextless" (
	pt1: [2]fixed_bcd.BCD($FRAC_DIGITS),
	sharedPt: [2]fixed_bcd.BCD(FRAC_DIGITS),
	pt2: [2]fixed_bcd.BCD(FRAC_DIGITS),
) -> bool {
	a := fixed_bcd.sub(sharedPt.x, pt1.x)
	b := fixed_bcd.sub(pt2.y, sharedPt.y)
	c := fixed_bcd.sub(sharedPt.y, pt1.y)
	d := fixed_bcd.sub(pt2.x, sharedPt.x)
	// When checking for collinearity with very large coordinate values
	// then ProductsAreEqual is more accurate than using CrossProduct.
	return fixed_bcd.equal(fixed_bcd.mul(a, b), fixed_bcd.mul(c, d))
}

@(private = "file")
SegmentsIntersectExclusive :: proc "contextless" (
	seg1a: [2]fixed_bcd.BCD($FRAC_DIGITS),
	seg1b: [2]fixed_bcd.BCD(FRAC_DIGITS),
	seg2a: [2]fixed_bcd.BCD(FRAC_DIGITS),
	seg2b: [2]fixed_bcd.BCD(FRAC_DIGITS),
) -> bool {
	zero :: fixed_bcd.BCD(FRAC_DIGITS){}
	dy1 := fixed_bcd.sub(seg1b.y, seg1a.y)
	dx1 := fixed_bcd.sub(seg1b.x, seg1a.x)
	dy2 := fixed_bcd.sub(seg2b.y, seg2a.y)
	dx2 := fixed_bcd.sub(seg2b.x, seg2a.x)
	cp := fixed_bcd.sub(fixed_bcd.mul(dy1, dx2), fixed_bcd.mul(dy2, dx1))
	if fixed_bcd.equal(cp, zero) do return false

	t := fixed_bcd.sub(
		fixed_bcd.mul(fixed_bcd.sub(seg1a.x, seg2a.x), dy2),
		fixed_bcd.mul(fixed_bcd.sub(seg1a.y, seg2a.y), dx2),
	)
	if fixed_bcd.equal(t, zero) do return false
	if fixed_bcd.greater(t, zero) {
		if fixed_bcd.less(cp, zero) || fixed_bcd.greater_than(t, cp) do return false
	} else if fixed_bcd.greater(cp, zero) || fixed_bcd.less_than(t, cp) do return false

	t = fixed_bcd.sub(
		fixed_bcd.mul(fixed_bcd.sub(seg1a.x, seg2a.x), dy1),
		fixed_bcd.mul(fixed_bcd.sub(seg1a.y, seg2a.y), dx1),
	)
	if fixed_bcd.equal(t, zero) do return false
	if fixed_bcd.greater(t, zero) do return fixed_bcd.greater(cp, zero) && fixed_bcd.less(t, cp)
	return fixed_bcd.less(cp, zero) && fixed_bcd.greater(t, cp)
}

@(private = "file")
CleanCollinear :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
	outrec: ^OutRec(FRAC_DIGITS, U),
) -> Clipper_Error {
	outrec_ := GetRealOutRec(outrec)
	if outrec_ == nil || outrec_.is_open do return nil

	if !IsValidClosedPath(outrec_.pts) {
		DisposeOutPts(ctx, outrec_)
		return nil
	}

	start_op := outrec_.pts
	op2 := start_op
	for {
		if IsCollinear(op2.prev.pt, op2.pt, op2.next.pt) &&
		   (op2.pt == op2.prev.pt ||
				   op2.pt == op2.next.pt ||
				   !ctx.preserve_collinear_ ||
				   linalg_ex.DotProduct(op2.prev.pt, op2.pt, op2.next.pt).i < 0) {

			if op2 == outrec_.pts do outrec_.pts = op2.prev

			op2 = DisposeOutPt(op2)

			if !IsValidClosedPath(op2) {
				DisposeOutPts(ctx, outrec_)
				return nil
			}

			start_op = op2
			continue
		}
		op2 = op2.next
		if op2 == start_op do break
	}
	FixSelfIntersects(ctx, outrec_) or_return
	return nil
}

@(private = "file")
Area_Double :: proc "contextless" (op: ^OutPt($FRAC_DIGITS, $U)) -> fixed_bcd.BCD(FRAC_DIGITS) {
	// Shoelace formula: https://en.wikipedia.org/wiki/Shoelace_formula
	result := fixed_bcd.BCD(FRAC_DIGITS){}
	op2 := op

	for {
		result = fixed_bcd.add(
			result,
			fixed_bcd.mul(
				fixed_bcd.add(op2.prev.pt.y, op2.pt.y),
				fixed_bcd.sub(op2.prev.pt.x, op2.pt.x),
			),
		)
		op2 = op2.next
		if op2 == op do break
	}

	return result // fixed_bcd.div(result, fixed_bcd.init_const(2, 0, 0, FRAC_DIGITS))
}

@(private = "file")
AreaTriangle :: proc "contextless" (
	pt1: [2]fixed_bcd.BCD($FRAC_DIGITS),
	pt2: [2]fixed_bcd.BCD(FRAC_DIGITS),
	pt3: [2]fixed_bcd.BCD(FRAC_DIGITS),
) -> fixed_bcd.BCD(FRAC_DIGITS) {
	term1 := fixed_bcd.mul(fixed_bcd.add(pt3.y, pt1.y), fixed_bcd.sub(pt3.x, pt1.x))
	term2 := fixed_bcd.mul(fixed_bcd.add(pt1.y, pt2.y), fixed_bcd.sub(pt1.x, pt2.x))
	term3 := fixed_bcd.mul(fixed_bcd.add(pt2.y, pt3.y), fixed_bcd.sub(pt2.x, pt3.x))
	return fixed_bcd.add(fixed_bcd.add(term1, term2), term3)
}

@(private = "file")
StartOpenPath :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
	e: ^Active(FRAC_DIGITS, U),
	pt: [2]fixed_bcd.BCD(FRAC_DIGITS),
) -> (
	out: ^OutPt(FRAC_DIGITS, U),
	err: Clipper_Error,
) {
	outrec := NewOutRec(ctx) or_return
	outrec.is_open = true

	if e.wind_dx > 0 {
		outrec.front_edge = e
		outrec.back_edge = nil
	} else {
		outrec.front_edge = nil
		outrec.back_edge = e
	}

	e.outrec = outrec

	op := new_clone(
		OutPt(FRAC_DIGITS, U){pt = pt, outrec = outrec},
		context.temp_allocator,
	) or_return
	op.next = op
	op.prev = op
	outrec.pts = op
	return op, nil
}

@(private = "file")
FindEdgeWithMatchingLocMin :: proc "contextless" (
	e: ^Active($FRAC_DIGITS, $U),
) -> ^Active(FRAC_DIGITS, U) {
	result := e.next_in_ael
	for result != nil {
		if result.local_min == e.local_min do return result
		else if !IsHorizontal(result) && e.bot != result.bot do result = nil
		else do result = result.next_in_ael
	}

	result = e.prev_in_ael
	for result != nil {
		if result.local_min == e.local_min do return result
		else if !IsHorizontal(result) && e.bot != result.bot do return nil
		else do result = result.prev_in_ael
	}
	return result
}

@(private = "file")
SwapOutrecs :: proc "contextless" (
	ctx: ^Context($FRAC_DIGITS, $U),
	e1: ^Active(FRAC_DIGITS, U),
	e2: ^Active(FRAC_DIGITS, U),
) {
	or1 := e1.outrec
	or2 := e2.outrec

	if or1 == or2 {
		e := or1.front_edge
		or1.front_edge = or1.back_edge
		or1.back_edge = e
		return
	}

	if or1 != nil {
		if e1 == or1.front_edge do or1.front_edge = e2
		else do or1.back_edge = e2
	}

	if or2 != nil {
		if e2 == or2.front_edge do or2.front_edge = e1
		else do or2.back_edge = e1
	}

	e1.outrec = or2
	e2.outrec = or1
}


@(private = "file")
IntersectEdges :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
	e1: ^Active(FRAC_DIGITS, U),
	e2: ^Active(FRAC_DIGITS, U),
	pt: [2]fixed_bcd.BCD(FRAC_DIGITS),
) -> Clipper_Error {
	// MANAGE OPEN PATH INTERSECTIONS SEPARATELY ...
	if ctx.has_open_paths_ && (IsOpenActive(e1) || IsOpenActive(e2)) {
		if IsOpenActive(e1) && IsOpenActive(e2) do return nil
		edge_o, edge_c: ^Active(FRAC_DIGITS, U)

		if IsOpenActive(e1) {
			edge_o = e1
			edge_c = e2
		} else {
			edge_o = e2
			edge_c = e1
		}

		if IsJoined(edge_c) do Split(ctx, edge_c, pt) or_return // needed for safety

		if abs(edge_c.wind_cnt) != 1 do return nil

		#partial switch ctx.clip_type_ {
		case .Union:
			if !IsHotEdge(edge_c) do return nil
		case:
			if edge_c.local_min.polytype == .Subject do return nil
		}

		#partial switch ctx.fill_rule_ {
		case .Positive:
			if edge_c.wind_cnt != 1 do return nil
		case .Negative:
			if edge_c.wind_cnt != -1 do return nil
		case:
			if abs(edge_c.wind_cnt) != 1 do return nil
		}

		if IsHotEdge(edge_o) { 	//toggle contribution ...
			AddOutPt(ctx, edge_o, pt) or_return

			if IsFront(edge_o) do edge_o.outrec.front_edge = nil
			else do edge_o.outrec.back_edge = nil
			edge_o.outrec = nil

		} else if fixed_bcd.equal(pt, edge_o.local_min.vertex.pt) &&
		   !IsOpenEndVertex(edge_o.local_min.vertex) { 	// horizontal edges can pass under open paths at a LocMins
			//find the other side of the LocMin and
			//if it's 'hot' join up with it ...
			e3 := FindEdgeWithMatchingLocMin(edge_o)

			if e3 != nil && IsHotEdge(e3) {
				edge_o.outrec = e3.outrec
				if edge_o.wind_dx > 0 do SetSides(e3.outrec, edge_o, e3)
				else do SetSides(e3.outrec, e3, edge_o)

				return nil
			} else {
				StartOpenPath(ctx, edge_o, pt) or_return
			}
		} else {
			StartOpenPath(ctx, edge_o, pt) or_return
		}
		return nil
	} // end of an open path intersection

	//MANAGING CLOSED PATHS FROM HERE ON

	if IsJoined(e1) do Split(ctx, e1, pt) or_return
	if IsJoined(e2) do Split(ctx, e2, pt) or_return

	//UPDATE WINDING COUNTS...

	old_e1_windcnt, old_e2_windcnt: i32
	if e1.local_min.polytype == e2.local_min.polytype {
		if ctx.fill_rule_ == .EvenOdd {
			old_e1_windcnt = e1.wind_cnt
			e1.wind_cnt = e2.wind_cnt
			e2.wind_cnt = old_e1_windcnt

		} else {
			if e1.wind_cnt + e2.wind_dx == 0 do e1.wind_cnt = -e1.wind_cnt
			else do e1.wind_cnt += e2.wind_dx

			if e2.wind_cnt - e1.wind_dx == 0 do e2.wind_cnt = -e2.wind_cnt
			else do e2.wind_cnt -= e1.wind_dx

		}
	} else {
		if ctx.fill_rule_ != .EvenOdd {
			e1.wind_cnt2 += e2.wind_dx
			e2.wind_cnt2 -= e1.wind_dx
		} else {
			e1.wind_cnt2 = (e1.wind_cnt2 == 0 ? 1 : 0)
			e2.wind_cnt2 = (e2.wind_cnt2 == 0 ? 1 : 0)
		}
	}

	switch ctx.fill_rule_ {
	case .EvenOdd, .NonZero:
		old_e1_windcnt = abs(e1.wind_cnt)
		old_e2_windcnt = abs(e2.wind_cnt)
	case .Positive:
		old_e1_windcnt = e1.wind_cnt
		old_e2_windcnt = e2.wind_cnt
	case .Negative:
		old_e1_windcnt = -e1.wind_cnt
		old_e2_windcnt = -e2.wind_cnt
	}

	e1_windcnt_in_01 := old_e1_windcnt == 0 || old_e1_windcnt == 1
	e2_windcnt_in_01 := old_e2_windcnt == 0 || old_e2_windcnt == 1
	if (!IsHotEdge(e1) && !e1_windcnt_in_01) || (!IsHotEdge(e2) && !e2_windcnt_in_01) do return nil

	// NOW PROCESS THE INTERSECTION ...
	if IsHotEdge(e1) && IsHotEdge(e2) {
		if (old_e1_windcnt != 0 && old_e1_windcnt != 1) ||
		   (old_e2_windcnt != 0 && old_e2_windcnt != 1) ||
		   (e1.local_min.polytype != e2.local_min.polytype && ctx.clip_type_ != .Xor) {
			AddLocalMaxPoly(ctx, e1, e2, pt) or_return

		} else if IsFront(e1) || (e1.outrec == e2.outrec) {
			//this 'else if' condition isn't strictly needed but
			//it's sensible to split polygons that only touch at
			//a common vertex (not at common edges).
			AddLocalMaxPoly(ctx, e1, e2, pt) or_return
			AddLocalMinPoly(ctx, e1, e2, pt) or_return
		} else {
			AddOutPt(ctx, e1, pt) or_return
			AddOutPt(ctx, e2, pt) or_return
			SwapOutrecs(ctx, e1, e2)
		}

	} else if IsHotEdge(e1) {
		AddOutPt(ctx, e1, pt) or_return
		SwapOutrecs(ctx, e1, e2)

	} else if IsHotEdge(e2) {
		AddOutPt(ctx, e2, pt) or_return
		SwapOutrecs(ctx, e1, e2)

	} else {
		e1_wc2, e2_wc2: i32

		switch ctx.fill_rule_ {
		case .EvenOdd, .NonZero:
			e1_wc2 = abs(e1.wind_cnt2)
			e2_wc2 = abs(e2.wind_cnt2)
		case .Positive:
			e1_wc2 = e1.wind_cnt2
			e2_wc2 = e2.wind_cnt2
		case .Negative:
			e1_wc2 = -e1.wind_cnt2
			e2_wc2 = -e2.wind_cnt2
		}

		if !IsSamePolyType(e1, e2) {
			AddLocalMinPoly(ctx, e1, e2, pt, false) or_return

		} else if old_e1_windcnt == 1 && old_e2_windcnt == 1 {
			#partial switch ctx.clip_type_ {
			case .Union:
				if e1_wc2 <= 0 && e2_wc2 <= 0 do AddLocalMinPoly(ctx, e1, e2, pt, false) or_return

			case .Difference:
				if (GetPolyType(e1) == .Clip && e1_wc2 > 0 && e2_wc2 > 0) ||
				   (GetPolyType(e1) == .Subject && e1_wc2 <= 0 && e2_wc2 <= 0) {
					AddLocalMinPoly(ctx, e1, e2, pt, false) or_return
				}

			case .Xor:
				AddLocalMinPoly(ctx, e1, e2, pt, false) or_return

			case:
				if e1_wc2 > 0 && e2_wc2 > 0 do AddLocalMinPoly(ctx, e1, e2, pt, false) or_return
			}
		}
	}
	return nil
}

@(private = "file")
IsSamePolyType :: proc "contextless" (
	e1: ^Active($FRAC_DIGITS, $U),
	e2: ^Active(FRAC_DIGITS, U),
) -> bool {
	return e1.local_min.polytype == e2.local_min.polytype
}

@(private = "file")
PerpendicDistFromLineSqrd :: proc(
	pt: [2]fixed_bcd.BCD($FRAC_DIGITS),
	line_a: [2]fixed_bcd.BCD(FRAC_DIGITS),
	line_b: [2]fixed_bcd.BCD(FRAC_DIGITS),
) -> (
	fixed_bcd.BCD(FRAC_DIGITS),
	fixed_bcd.BCD(FRAC_DIGITS),
) {
	//perpendicular distance of point (x³,y³) = (Ax³ + By³ + C)/Sqrt(A² + B²)
	//see https://en.wikipedia.org/wiki/Perpendicular_distance
	dx := fixed_bcd.sub(line_b.x, line_a.x)
	dy := fixed_bcd.sub(line_b.y, line_a.y)
	if dx.i == 0 && dy.i == 0 do return fixed_bcd.BCD(FRAC_DIGITS){i = 0}, fixed_bcd.init_const(1, 0, 0, FRAC_DIGITS)

	den := fixed_bcd.add(fixed_bcd.mul(dx, dx), fixed_bcd.mul(dy, dy))

	num := fixed_bcd.sub(
		fixed_bcd.mul(fixed_bcd.sub(pt.x, line_a.x), dy),
		fixed_bcd.mul(fixed_bcd.sub(pt.y, line_a.y), dx),
	)

	return fixed_bcd.mul(num, num), den
}

@(private = "file")
IsValidAelOrder :: proc "contextless" (
	resident: ^Active($FRAC_DIGITS, $U),
	newcomer: ^Active(FRAC_DIGITS, U),
) -> bool {
	if !fixed_bcd.equal(newcomer.curr_x, resident.curr_x) {
		return fixed_bcd.greater(newcomer.curr_x, resident.curr_x)
	}
	//get the turning direction  a1.top, a2.bot, a2.top
	i := linalg_ex.CrossProductSign(resident.top, newcomer.bot, newcomer.top)
	if i != 0 do return i < 0

	//edges must be collinear to get here
	//for starting open paths, place them according to
	//the direction they're about to turn
	if !IsMaxima(resident.vertex_top) && fixed_bcd.greater(resident.top.y, newcomer.top.y) {
		return linalg_ex.CrossProductSign(newcomer.bot, resident.top, NextVertex(resident).pt) <= 0
	} else if !IsMaxima(newcomer.vertex_top) && fixed_bcd.greater(newcomer.top.y, resident.top.y) {
		return linalg_ex.CrossProductSign(newcomer.bot, newcomer.top, NextVertex(newcomer).pt) >= 0
	}

	y := newcomer.bot.y
	newcomer_is_left := newcomer.is_left_bound

	if resident.bot.y != y || resident.local_min.vertex.pt.y != y {
		return newcomer_is_left
	} else if resident.is_left_bound != newcomer_is_left { 	//resident must also have just been inserted
		return newcomer_is_left
	} else if IsCollinear(PrevPrevVertex(resident).pt, resident.bot, resident.top) {
		return true
	} else { 	// compare turning direction of the alternate bound
		return(
			(linalg_ex.CrossProductSign(
					PrevPrevVertex(resident).pt,
					newcomer.bot,
					PrevPrevVertex(newcomer).pt,
				) >
				0) ==
			newcomer_is_left \
		)
	}
}

@(private = "file")
SwapPositionsInAEL :: proc "contextless" (
	ctx: ^Context($FRAC_DIGITS, $U),
	e1: ^Active(FRAC_DIGITS, U),
	e2: ^Active(FRAC_DIGITS, U),
) {
	// Precondition: e1 is immediately to the left of e2.
	next := e2.next_in_ael
	if next != nil do next.prev_in_ael = e1
	prev := e1.prev_in_ael
	if prev != nil do prev.next_in_ael = e2
	e2.prev_in_ael = prev
	e2.next_in_ael = e1
	e1.prev_in_ael = e2
	e1.next_in_ael = next
	if e2.prev_in_ael == nil do ctx.actives_ = e2
}

@(private = "file")
InsertScanline :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
	y: fixed_bcd.BCD(FRAC_DIGITS),
) -> (
	err: Clipper_Error,
) {
	pq.push(&ctx.scanline_list_, y) or_return
	return
}

@(private = "file")
PushHorz :: proc "contextless" (ctx: ^Context($FRAC_DIGITS, $U), e: ^Active(FRAC_DIGITS, U)) {
	e.next_in_sel = ctx.sel_
	ctx.sel_ = e
}

@(private = "file")
PopHorz :: proc "contextless" (ctx: ^Context($FRAC_DIGITS, $U)) -> ^Active(FRAC_DIGITS, U) {
	if ctx.sel_ == nil do return nil
	res := ctx.sel_
	ctx.sel_ = ctx.sel_.next_in_sel
	return res
}


@(private = "file")
GetCurrYMaximaVertex_Open :: proc(horz: ^Active($FRAC_DIGITS, $U)) -> ^Vertex(FRAC_DIGITS, U) {
	result := horz.vertex_top

	if horz.wind_dx > 0 {
		for fixed_bcd.equal(result.next.pt.y, result.pt.y) &&
		    (result.flags & VertexFlags_Set{.OpenEnd, .LocalMax}) == {} {
			result = result.next
		}
	} else {
		for fixed_bcd.equal(result.prev.pt.y, result.pt.y) &&
		    (result.flags & VertexFlags_Set{.OpenEnd, .LocalMax}) == {} {
			result = result.prev
		}
	}
	if !IsMaxima(result) do return nil // not a maxima
	return result
}

@(private = "file")
GetCurrYMaximaVertex :: proc(horz: ^Active($FRAC_DIGITS, $U)) -> ^Vertex(FRAC_DIGITS, U) {
	result := horz.vertex_top
	if horz.wind_dx > 0 {
		for fixed_bcd.equal(result.next.pt.y, result.pt.y) do result = result.next
	} else {
		for fixed_bcd.equal(result.prev.pt.y, result.pt.y) do result = result.prev
	}
	if !IsMaxima(result) do return nil // not a maxima
	return result
}

@(private = "file")
ResetHorzDirection :: proc(
	horz: ^Active($FRAC_DIGITS, $U),
	vertex_max: ^Vertex(FRAC_DIGITS, U),
) -> (
	bool,
	fixed_bcd.BCD(FRAC_DIGITS),
	fixed_bcd.BCD(FRAC_DIGITS),
) {
	if (horz.bot.x == horz.top.x) {
		//the horizontal edge is going nowhere ...
		e := horz.next_in_ael

		for (e != nil) && (e.vertex_top != vertex_max) do e = e.next_in_ael
		return e != nil, horz.curr_x, horz.curr_x
	} else if (fixed_bcd.less(horz.curr_x, horz.top.x)) {
		return true, horz.curr_x, horz.top.x
	} else {
		return false, horz.top.x, horz.curr_x // right to left
	}
}

@(private = "file")
AddTrialHorzJoin :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
	op: ^OutPt(FRAC_DIGITS, U),
) -> Clipper_Error {
	if op.outrec.is_open do return nil
	non_zero_append(&ctx.horz_seg_list_, HorzSegment(FRAC_DIGITS, U){left_op = op}) or_return
	return nil
}

@(private = "file")
GetLastOp :: proc(hot_edge: ^Active($FRAC_DIGITS, $U)) -> ^OutPt(FRAC_DIGITS, U) {
	res := hot_edge.outrec.pts
	if hot_edge != hot_edge.outrec.front_edge do res = res.next
	return res
}

/*******************************************************************************
* Notes: Horizontal edges (HEs) at scanline intersections (ie at the top or    *
* bottom of a scanbeam) are processed as if layered.The order in which HEs     *
* are processed doesn't matter. HEs intersect with the bottom vertices of      *
* other HEs[#] and with non-horizontal edges [*]. Once these intersections     *
* are completed, intermediate HEs are 'promoted' to the next edge in their     *
* bounds, and they in turn may be intersected[%] by other HEs.                 *
*                                                                              *
* eg: 3 horizontals at a scanline:    /   |                     /           /  *
*              |                     /    |     (HE3)o ========%========== o   *
*              o ======= o(HE2)     /     |         /         /                *
*          o ============#=========*======*========#=========o (HE1)           *
*         /              |        /       |       /                            *
*******************************************************************************/
@(private = "file")
DoHorizontal :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
	horz: ^Active(FRAC_DIGITS, U),
) -> Clipper_Error {
	horz_is_open := IsOpenActive(horz)
	y := horz.bot.y
	vertex_max := horz_is_open ? GetCurrYMaximaVertex_Open(horz) : GetCurrYMaximaVertex(horz)

	is_left_to_right, horz_left, horz_right := ResetHorzDirection(horz, vertex_max)

	if IsHotEdge(horz) {
		op := AddOutPt(ctx, horz, [2]fixed_bcd.BCD(FRAC_DIGITS){horz.curr_x, y}) or_return
		AddTrialHorzJoin(ctx, op) or_return
	}

	for { 	// loop through consec. horizontal edges
		e: ^Active(FRAC_DIGITS, U)
		if is_left_to_right do e = horz.next_in_ael
		else do e = horz.prev_in_ael

		for e != nil {
			if e.vertex_top == vertex_max {
				if IsHotEdge(horz) && IsJoined(e) do Split(ctx, e, e.top) or_return
				if IsHotEdge(horz) {
					for horz.vertex_top != vertex_max {
						AddOutPt(ctx, horz, horz.top) or_return
						UpdateEdgeIntoAEL(ctx, horz) or_return
					}
					if is_left_to_right do AddLocalMaxPoly(ctx, horz, e, horz.top) or_return
					else do AddLocalMaxPoly(ctx, e, horz, horz.top) or_return
				}
				DeleteFromAEL(ctx, e)
				DeleteFromAEL(ctx, horz)
				return nil
			}

			// If horz is a maxima, keep going until we reach its maxima pair; else check break conditions
			if vertex_max != horz.vertex_top || IsOpenEnd(horz) {
				// otherwise stop when 'ae' is beyond the end of the horizontal line
				if (is_left_to_right && fixed_bcd.greater(e.curr_x, horz_right)) ||
				   (!is_left_to_right && fixed_bcd.less(e.curr_x, horz_left)) {
					break
				}

				if fixed_bcd.equal(e.curr_x, horz.top.x) && !IsHorizontal(e) {
					pt := NextVertex(horz).pt

					if is_left_to_right {
						//with open paths we'll only break once past horz's end
						if IsOpenActive(e) && !IsSamePolyType(e, horz) && !IsHotEdge(e) {
							if fixed_bcd.greater(TopX(e, pt.y), pt.x) do break
						} else if fixed_bcd.greater_than(TopX(e, pt.y), pt.x) do break // otherwise we'll only break when horz's outslope is greater than e's
					} else {
						if IsOpenActive(e) && !IsSamePolyType(e, horz) && !IsHotEdge(e) {
							if fixed_bcd.less(TopX(e, pt.y), pt.x) do break
						} else if fixed_bcd.less_than(TopX(e, pt.y), pt.x) do break
					}
				}
			}

			pt := [2]fixed_bcd.BCD(FRAC_DIGITS){e.curr_x, horz.bot.y}
			if is_left_to_right {
				IntersectEdges(ctx, horz, e, pt) or_return
				SwapPositionsInAEL(ctx, horz, e)
				CheckJoinLeft(ctx, e, pt) or_return
				horz.curr_x = e.curr_x
				e = horz.next_in_ael
			} else {
				IntersectEdges(ctx, e, horz, pt) or_return
				SwapPositionsInAEL(ctx, e, horz)
				CheckJoinRight(ctx, e, pt) or_return
				horz.curr_x = e.curr_x
				e = horz.prev_in_ael
			}

			if horz.outrec != nil do AddTrialHorzJoin(ctx, GetLastOp(horz)) or_return
		}

		// check if we've finished with (consecutive) horizontals ...
		if horz_is_open && IsOpenEnd(horz) { 	// open at top
			if IsHotEdge(horz) {
				AddOutPt(ctx, horz, horz.top) or_return
				if IsFront(horz) do horz.outrec.front_edge = nil
				else do horz.outrec.back_edge = nil
				horz.outrec = nil
			}
			DeleteFromAEL(ctx, horz)
			return nil
		} else if !fixed_bcd.equal(NextVertex(horz).pt.y, horz.top.y) do break

		//still more horizontals in bound to process ...
		if IsHotEdge(horz) do AddOutPt(ctx, horz, horz.top) or_return
		UpdateEdgeIntoAEL(ctx, horz) or_return
		is_left_to_right, horz_left, horz_right = ResetHorzDirection(horz, vertex_max)
	}

	if IsHotEdge(horz) {
		op := AddOutPt(ctx, horz, horz.top) or_return
		AddTrialHorzJoin(ctx, op) or_return
	}
	UpdateEdgeIntoAEL(ctx, horz) or_return
	return nil
}


@(private = "file")
DuplicateOp :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
	op: ^OutPt(FRAC_DIGITS, U),
	insert_after: bool,
) -> (
	result: ^OutPt(FRAC_DIGITS, U),
	err: Clipper_Error,
) {
	result = new_clone(
		OutPt(FRAC_DIGITS, U){pt = op.pt, outrec = op.outrec},
		context.temp_allocator,
	) or_return

	if insert_after {
		result.next = op.next
		result.next.prev = result
		result.prev = op
		op.next = result
	} else {
		result.prev = op.prev
		result.prev.next = result
		result.next = op
		op.prev = result
	}
	return result, nil
}

@(private = "file")
FixOutRecPts :: proc(outrec: ^OutRec($FRAC_DIGITS, $U)) {
	op := outrec.pts
	for {
		op.outrec = outrec
		op = op.next
		if op == outrec.pts do break
	}
}

@(private = "file")
PointInOpPolygon :: proc "contextless" (
	pt: [2]fixed_bcd.BCD($FRAC_DIGITS),
	op: ^OutPt(FRAC_DIGITS, $U),
) -> linalg_ex.PointInPolygonResult {
	if op == op.next || op.prev == op.next do return .Outside

	op2 := op
	for fixed_bcd.equal(op.pt.y, pt.y) {
		op = op.next
		if op == op2 do break
	}
	if fixed_bcd.equal(op.pt.y, pt.y) do return .Outside // not a proper polygon

	is_above := fixed_bcd.less(op.pt.y, pt.y)
	starting_above := is_above

	val := 0
	op2 = op.next
	for op2 != op {
		if is_above {
			for op2 != op && fixed_bcd.less(op2.pt.y, pt.y) do op2 = op2.next
		} else {
			for op2 != op && fixed_bcd.greater(op2.pt.y, pt.y) do op2 = op2.next
		}
		if op2 == op do break

		// must have touched or crossed the pt.Y horizontal
		// and this must happen an even number of times

		if fixed_bcd.equal(op2.pt.y, pt.y) {

			if fixed_bcd.equal(op2.pt.x, pt.x) ||
			   (fixed_bcd.equal(op2.pt.y, op2.prev.pt.y) &&
					   fixed_bcd.less(pt.x, op2.prev.pt.x) != fixed_bcd.less(pt.x, op2.pt.x)) {
				return .On
			}
			op2 = op2.next
			if op2 == op do break
			continue
		}

		if fixed_bcd.less(pt.x, op2.pt.x) && fixed_bcd.less(pt.x, op2.prev.pt.x) {
			// do nothing, only interested in edges crossing on the left
		} else if fixed_bcd.greater(pt.x, op2.prev.pt.x) && fixed_bcd.greater(pt.x, op2.pt.x) {
			val = 1 - val
		} else {
			i := linalg_ex.CrossProductSign(op2.prev.pt, op2.pt, pt)
			if i == 0 do return .On
			if (i < 0) == is_above do val = 1 - val
		}
		is_above = !is_above
		op2 = op2.next
	}

	if is_above != starting_above {
		i := linalg_ex.CrossProductSign(op2.prev.pt, op2.pt, pt)
		if i == 0 do return .On
		if (i < 0) == is_above do val = 1 - val
	}

	if val == 0 do return .Outside
	return .Inside
}

//? only for polytree
// @(private = "file")
// GetCleanPath :: proc(
// 	op: ^OutPt($FRAC_DIGITS),
// ) -> (
// 	[][2]fixed_bcd.BCD(FRAC_DIGITS),
// 	Clipper_Error,
// ) {
// 	path := make([dynamic][2]fixed_bcd.BCD(FRAC_DIGITS), context.temp_allocator) or_return
// 	op2 := op

// 	for op2.next != op &&
// 	    ((fixed_bcd.equal(op2.pt.x, op2.next.pt.x) && fixed_bcd.equal(op2.pt.x, op2.prev.pt.x)) ||
// 			    (fixed_bcd.equal(op2.pt.y, op2.next.pt.y) &&
// 					    fixed_bcd.equal(op2.pt.y, op2.prev.pt.y))) {
// 		op2 = op2.next
// 	}
// 	non_zero_append(&path, op2.pt) or_return
// 	prev_op := op2
// 	op2 = op2.next

// 	for op2 != op {
// 		if (!fixed_bcd.equal(op2.pt.x, op2.next.pt.x) ||
// 			   !fixed_bcd.equal(op2.pt.x, prev_op.pt.x)) &&
// 		   (!fixed_bcd.equal(op2.pt.y, op2.next.pt.y) ||
// 				   !fixed_bcd.equal(op2.pt.y, prev_op.pt.y)) {

// 			non_zero_append(&path, op2.pt) or_return
// 			prev_op = op2
// 		}
// 		op2 = op2.next
// 	}
// 	return path[:], nil
// }


// @(private = "file")
// Path2ContainsPath1 :: proc(
// 	op1: ^OutPt($FRAC_DIGITS),
// 	op2: ^OutPt(FRAC_DIGITS),
// ) -> (
// 	bool,
// 	Clipper_Error,
// ) {
// 	pip := linalg_ex.PointInPolygonResult.On
// 	op := op1
// 	for {
// 		switch linalg_ex.PointInOpPolygon(op.pt, op2) {
// 		case .Outside:
// 			if pip == .Outside do return false, nil
// 			pip = .Outside
// 		case .Inside:
// 			if pip == .Inside do return true, nil
// 			pip = .Inside
// 		case .On:
// 			break
// 		}
// 		op = op.next
// 		if op == op1 do break
// 	}
// 	return Path2ContainsPath1(GetCleanPath(op1) or_return, GetCleanPath(op2) or_return)
// }

@(private = "file")
SetHorzSegHeadingForward :: proc "contextless" (
	hs: ^HorzSegment($FRAC_DIGITS, $U),
	op_p: ^OutPt(FRAC_DIGITS, U),
	op_n: ^OutPt(FRAC_DIGITS, U),
) -> bool {
	if fixed_bcd.equal(op_p.pt.x, op_n.pt.x) do return false
	if fixed_bcd.less(op_p.pt.x, op_n.pt.x) {
		hs.left_op = op_p
		hs.right_op = op_n
		hs.left_to_right = true
	} else {
		hs.left_op = op_n
		hs.right_op = op_p
		hs.left_to_right = false
	}
	return true
}

@(private = "file")
UpdateHorzSegment :: proc "contextless" (
	ctx: ^Context($FRAC_DIGITS, $U),
	hs: ^HorzSegment(FRAC_DIGITS, U),
) -> bool {
	op := hs.left_op
	outrec := GetRealOutRec(op.outrec)
	outrec_has_edges := outrec.front_edge != nil
	curr_y := op.pt.y
	op_p := op
	op_n := op

	if outrec_has_edges {
		op_a := outrec.pts
		op_z := op_a.next
		for op_p != op_z && op_p.prev.pt.y == curr_y do op_p = op_p.prev
		for op_n != op_a && op_n.next.pt.y == curr_y do op_n = op_n.next
	} else {
		for op_p.prev != op_n && op_p.prev.pt.y == curr_y do op_p = op_p.prev
		for op_n.next != op_p && op_n.next.pt.y == curr_y do op_n = op_n.next
	}

	result := SetHorzSegHeadingForward(hs, op_p, op_n) && hs.left_op.horz == nil
	if result {
		hs.left_op.horz = hs
	} else {
		hs.right_op = nil // (for sorting)
	}
	return result
}

@(private = "file")
AddNewIntersectNode :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
	e1: ^Active(FRAC_DIGITS, U),
	e2: ^Active(FRAC_DIGITS, U),
	top_y: fixed_bcd.BCD(FRAC_DIGITS),
) -> (
	err: Clipper_Error,
) {
	ok, pt := GetLineIntersectPt(e1.bot, e1.top, e2.bot, e2.top)
	if !ok {
		pt = [2]fixed_bcd.BCD(FRAC_DIGITS){e1.curr_x, top_y}
	}
	if fixed_bcd.greater(pt.y, ctx.bot_y_) || fixed_bcd.less(pt.y, top_y) {
		abs_dx1 := e1.dx
		if abs_dx1.i < 0 do abs_dx1.i = -abs_dx1.i
		abs_dx2 := e2.dx
		if abs_dx2.i < 0 do abs_dx2.i = -abs_dx2.i

		if fixed_bcd.less(pt.y, top_y) do pt.y = top_y
		else do pt.y = ctx.bot_y_

		// Clamp intersection to the scanbeam and project with the less steep edge.
		if fixed_bcd.less(abs_dx1, abs_dx2) do pt.x = TopX(e1, pt.y)
		else do pt.x = TopX(e2, pt.y)
	}

	node := IntersectNode(FRAC_DIGITS, U) {
		pt    = pt,
		edge1 = e1,
		edge2 = e2,
	}
	non_zero_append(&ctx.intersect_nodes_, node) or_return
	return
}

@(private = "file")
ExtractFromSEL :: proc "contextless" (e: ^Active($FRAC_DIGITS, $U)) -> ^Active(FRAC_DIGITS, U) {
	res := e.next_in_sel
	if res != nil do res.prev_in_sel = e.prev_in_sel
	e.prev_in_sel.next_in_sel = res
	return res
}

@(private = "file")
Insert1Before2InSEL :: proc "contextless" (
	e1: ^Active($FRAC_DIGITS, $U),
	e2: ^Active(FRAC_DIGITS, U),
) {
	e1.prev_in_sel = e2.prev_in_sel
	if e1.prev_in_sel != nil do e1.prev_in_sel.next_in_sel = e1
	e1.next_in_sel = e2
	e2.prev_in_sel = e1
}

@(private = "file")
BuildIntersectList :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
	top_y: fixed_bcd.BCD(FRAC_DIGITS),
) -> (
	res: bool,
	err: Clipper_Error,
) {
	if ctx.actives_ == nil || ctx.actives_.next_in_ael == nil do return

	//Calculate edge positions at the top of the current scanbeam, and from this
	//we will determine the intersections required to reach these new positions.
	AdjustCurrXAndCopyToSEL(ctx, top_y)
	//Find all edge intersections in the current scanbeam using a stable merge
	//sort that ensures only adjacent edges are intersecting. Intersect info is
	//stored in FIntersectList ready to be processed in ProcessIntersectList.
	//Re merge sorts see https://stackoverflow.com/a/46319131/359538

	left := ctx.sel_
	for left != nil && left.jump != nil {
		prev_base: ^Active(FRAC_DIGITS, U) = nil

		for left != nil && left.jump != nil {
			curr_base := left
			right := left.jump
			l_end := right
			r_end := right.jump

			left.jump = r_end
			for left != l_end && right != r_end {

				if fixed_bcd.less(right.curr_x, left.curr_x) {

					tmp := right.prev_in_sel
					for {
						AddNewIntersectNode(ctx, tmp, right, top_y) or_return
						if tmp == left do break
						tmp = tmp.prev_in_sel
					}

					tmp = right
					right = ExtractFromSEL(tmp)
					l_end = right
					Insert1Before2InSEL(tmp, left)

					if left == curr_base {
						curr_base = tmp
						curr_base.jump = r_end
						if prev_base == nil do ctx.sel_ = curr_base
						else do prev_base.jump = curr_base
					}
				} else {
					left = left.next_in_sel
				}
			}
			prev_base = curr_base
			left = r_end
		}
		left = ctx.sel_
	}
	return len(ctx.intersect_nodes_) > 0, nil
}


@(private = "file")
EdgesAdjacentInAEL :: proc "contextless" (node: IntersectNode($FRAC_DIGITS, $U)) -> bool {
	return node.edge1.next_in_ael == node.edge2 || node.edge1.prev_in_ael == node.edge2
}

@(private = "file")
ProcessIntersectList :: proc(ctx: ^Context($FRAC_DIGITS, $U)) -> (err: Clipper_Error) {
	//We now have a list of intersections required so that edges will be
	//correctly positioned at the top of the scanbeam. However, it's important
	//that edge intersections are processed from the bottom up, but it's also
	//crucial that intersections only occur between adjacent edges.

	//First we do a quicksort so intersections proceed in a bottom up order ...
	slice.sort_by(
		ctx.intersect_nodes_[:],
		proc(a, b: IntersectNode(FRAC_DIGITS, U)) -> bool {
			//note different inequality tests ...
			return(
				a.pt.y == b.pt.y ? fixed_bcd.less(a.pt.x, b.pt.x) : fixed_bcd.greater(a.pt.y, b.pt.y) \
			)
		},
	)
	//Now as we process these intersections, we must sometimes adjust the order
	//to ensure that intersecting edges are always adjacent ...

	for i in 0 ..< len(ctx.intersect_nodes_) {
		node := &ctx.intersect_nodes_[i]
		if !EdgesAdjacentInAEL(node^) {
			j := i + 1
			for j < len(ctx.intersect_nodes_) && !EdgesAdjacentInAEL(ctx.intersect_nodes_[j]) {
				j += 1
			}

			ctx.intersect_nodes_[i], ctx.intersect_nodes_[j] =
				ctx.intersect_nodes_[j], ctx.intersect_nodes_[i]
			node = &ctx.intersect_nodes_[i]
		}

		IntersectEdges(ctx, node.edge1, node.edge2, node.pt) or_return
		SwapPositionsInAEL(ctx, node.edge1, node.edge2)
		node.edge1.curr_x = node.pt.x
		node.edge2.curr_x = node.pt.x
		CheckJoinLeft(ctx, node.edge2, node.pt, true) or_return
		CheckJoinRight(ctx, node.edge1, node.pt, true) or_return
	}
	return
}

@(private = "file")
DoMaxima :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
	e: ^Active(FRAC_DIGITS, U),
) -> (
	out: ^Active(FRAC_DIGITS, U),
	err: Clipper_Error,
) {
	prev_e := e.prev_in_ael
	next_e := e.next_in_ael

	if IsOpenEnd(e) {
		if IsHotEdge(e) do AddOutPt(ctx, e, e.top) or_return

		if !IsHorizontal(e) {
			if IsHotEdge(e) {
				if IsFront(e) do e.outrec.front_edge = nil
				else do e.outrec.back_edge = nil
				e.outrec = nil
			}
			DeleteFromAEL(ctx, e)
		}
		return next_e, nil
	}

	max_pair := GetMaximaPair(e)
	if max_pair == nil do return next_e, nil // eMaxPair is horizontal

	if IsJoined(e) do Split(ctx, e, e.top) or_return
	if IsJoined(max_pair) do Split(ctx, max_pair, max_pair.top) or_return

	// only non-horizontal maxima here.
	// process any edges between maxima pair ...
	for next_e != max_pair {
		IntersectEdges(ctx, e, next_e, e.top) or_return
		SwapPositionsInAEL(ctx, e, next_e)
		next_e = e.next_in_ael
	}

	if IsOpenActive(e) {
		if IsHotEdge(e) do AddLocalMaxPoly(ctx, e, max_pair, e.top) or_return
		DeleteFromAEL(ctx, max_pair)
		DeleteFromAEL(ctx, e)
		return prev_e != nil ? prev_e.next_in_ael : ctx.actives_, nil
	}

	// e.next_in_ael== max_pair ...
	if IsHotEdge(e) do AddLocalMaxPoly(ctx, e, max_pair, e.top) or_return

	DeleteFromAEL(ctx, e)
	DeleteFromAEL(ctx, max_pair)
	return prev_e != nil ? prev_e.next_in_ael : ctx.actives_, nil
}

@(private = "file")
ConvertHorzSegsToJoins :: proc(ctx: ^Context($FRAC_DIGITS, $U)) -> Clipper_Error {
	j := 0
	for i in 0 ..< len(ctx.horz_seg_list_) {
		if UpdateHorzSegment(ctx, &ctx.horz_seg_list_[i]) do j += 1
	}
	if j < 2 do return nil

	slice.stable_sort_by(ctx.horz_seg_list_[:], proc(a, b: HorzSegment(FRAC_DIGITS, U)) -> bool {
		return(
			a.right_op != nil || b.right_op != nil ? a.right_op != nil : fixed_bcd.less(b.left_op.pt.x, a.left_op.pt.x) \
		)
	})

	for i in 0 ..< j - 1 {
		hs1 := &ctx.horz_seg_list_[i]

		for k in i + 1 ..< j {
			hs2 := &ctx.horz_seg_list_[k]
			if fixed_bcd.greater_than(hs2.left_op.pt.x, hs1.right_op.pt.x) ||
			   hs2.left_to_right == hs1.left_to_right ||
			   fixed_bcd.less_than(hs2.right_op.pt.x, hs1.left_op.pt.x) {
				continue
			}

			curr_y := hs1.left_op.pt.y
			if hs1.left_to_right {
				for hs1.left_op.next.pt.y == curr_y &&
				    fixed_bcd.less_than(hs1.left_op.next.pt.x, hs2.left_op.pt.x) {
					hs1.left_op = hs1.left_op.next
				}
				for hs2.left_op.prev.pt.y == curr_y &&
				    fixed_bcd.less_than(hs2.left_op.prev.pt.x, hs1.left_op.pt.x) {
					hs2.left_op = hs2.left_op.prev
				}

				non_zero_append(
					&ctx.horz_join_list_,
					HorzJoin(FRAC_DIGITS, U) {
						op1 = DuplicateOp(ctx, hs1.left_op, true) or_return,
						op2 = DuplicateOp(ctx, hs2.left_op, false) or_return,
					},
				) or_return
			} else {
				for hs1.left_op.prev.pt.y == curr_y &&
				    fixed_bcd.less_than(hs1.left_op.prev.pt.x, hs2.left_op.pt.x) {
					hs1.left_op = hs1.left_op.prev
				}
				for hs2.left_op.next.pt.y == curr_y &&
				    fixed_bcd.less_than(hs2.left_op.next.pt.x, hs1.left_op.pt.x) {
					hs2.left_op = hs2.left_op.next
				}

				non_zero_append(
					&ctx.horz_join_list_,
					HorzJoin(FRAC_DIGITS, U) {
						op1 = DuplicateOp(ctx, hs2.left_op, true) or_return,
						op2 = DuplicateOp(ctx, hs1.left_op, false) or_return,
					},
				) or_return
			}
		}
	}
	return nil
}

@(private = "file")
DoIntersections :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
	top_y: fixed_bcd.BCD(FRAC_DIGITS),
) -> Clipper_Error {
	if BuildIntersectList(ctx, top_y) or_return {
		ProcessIntersectList(ctx) or_return
		clear(&ctx.intersect_nodes_)
	}
	return nil
}

@(private = "file")
DoTopOfScanbeam :: proc(
	ctx: ^Context($FRAC_DIGITS, $U),
	y: fixed_bcd.BCD(FRAC_DIGITS),
) -> Clipper_Error {
	ctx.sel_ = nil // sel_ is reused to flag horizontals (see PushHorz)
	e := ctx.actives_
	for e != nil {
		if fixed_bcd.equal(e.top.y, y) {
			e.curr_x = e.top.x
			if IsMaxima(e.vertex_top) {
				e = DoMaxima(ctx, e) or_return // TOP OF BOUND (MAXIMA)
				continue
			}

			if IsHotEdge(e) do AddOutPt(ctx, e, e.top) or_return
			UpdateEdgeIntoAEL(ctx, e) or_return
			if IsHorizontal(e) do PushHorz(ctx, e)
		} else {
			e.curr_x = TopX(e, y)
		}
		e = e.next_in_ael
	}
	return nil
}

@(private = "file")
ProcessHorzJoins :: proc(ctx: ^Context($FRAC_DIGITS, $U)) -> Clipper_Error {
	for i in 0 ..< len(ctx.horz_join_list_) {
		j := &ctx.horz_join_list_[i]
		or1 := GetRealOutRec(j.op1.outrec)
		or2 := GetRealOutRec(j.op2.outrec)
		op1b := j.op1.next
		op2b := j.op2.prev
		j.op1.next = j.op2
		j.op2.prev = j.op1
		op1b.prev = op2b
		op2b.next = op1b

		if or1 == or2 {
			// 'join' is really a split
			or2 = NewOutRec(ctx) or_return
			or2.pts = op1b
			FixOutRecPts(or2)

			if or1.pts.outrec == or2 {
				or1.pts = j.op1
				or1.pts.outrec = or1
			}
			or2.owner = or1
		} else {
			or2.pts = nil
			or2.owner = or1
		}
	}
	return nil
}

@(private = "file")
BuildPath64 :: proc(
	op: ^OutPt($FRAC_DIGITS, $U),
	reverse: bool,
	$is_open: bool,
	path: ^[dynamic][dynamic][2]fixed_bcd.BCD(FRAC_DIGITS),
) -> Clipper_Error {
	if op == nil || op.next == op do return nil
	when !is_open {
		if op.next == op.prev do return nil
	}
	op := op

	non_zero_append(
		path,
		make([dynamic][2]fixed_bcd.BCD(FRAC_DIGITS), context.temp_allocator) or_return,
	) or_return

	op2: ^OutPt(FRAC_DIGITS, U)
	if (reverse) {
		op2 = op.prev
	} else {
		op = op.next
		op2 = op.next
	}
	lastPt := op.pt
	non_zero_append(&path[len(path) - 1], lastPt) or_return

	for op2 != op {
		if (op2.pt != lastPt) {
			lastPt = op2.pt
			non_zero_append(&path[len(path) - 1], lastPt) or_return
		}
		if (reverse) do op2 = op2.prev
		else do op2 = op2.next
	}
	return nil
}

//
//public
//

@(private = "file")
BooleanOp_Impl :: proc(
	clip_type: ClipType,
	$FRAC_DIGITS: int,
	subjects: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
	clips: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
	opens: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
	fill_rule: FillRule = .Positive,
	allocator := context.allocator,
) -> (
	res: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
	res_open: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
	err: Clipper_Error,
) {
	E :: struct {}
	res, res_open, _, _ = BooleanOpCustomData_Impl(
		clip_type,
		FRAC_DIGITS,
		subjects,
		clips,
		opens,
		E,
		subjects == nil ? nil : [][]E{},
		clips == nil ? nil : [][]E{},
		opens == nil ? nil : [][]E{},
		fill_rule,
		allocator,
	) or_return
	return
}

@(private = "file")
BooleanOpCustomData_Impl :: proc(
	clip_type: ClipType,
	$FRAC_DIGITS: int,
	subjects: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
	clips: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
	opens: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
	$U: typeid,
	extra_subjects: [][]U,
	extra_clips: [][]U,
	extra_opens: [][]U,
	fill_rule: FillRule = .Positive,
	allocator := context.allocator,
) -> (
	res: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
	res_open: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
	res_extra: [][]U,
	res_open_extra: [][]U,
	err: Clipper_Error,
) {
	ctx := Context(FRAC_DIGITS, U) {
		vertex_lists_       = make(
			[dynamic]^Vertex(FRAC_DIGITS, U),
			context.temp_allocator,
		) or_return,
		minima_list_        = make(
			[dynamic]^LocalMinima(FRAC_DIGITS, U),
			context.temp_allocator,
		) or_return,
		out                 = make(
			[dynamic][dynamic]fixed_bcd.BCD(FRAC_DIGITS),
			context.temp_allocator,
		) or_return,
		outrec_list_        = make(
			[dynamic]^OutRec(FRAC_DIGITS, U),
			context.temp_allocator,
		) or_return,
		horz_seg_list_      = make(
			[dynamic]HorzSegment(FRAC_DIGITS, U),
			context.temp_allocator,
		) or_return,
		horz_join_list_     = make(
			[dynamic]HorzJoin(FRAC_DIGITS, U),
			context.temp_allocator,
		) or_return,
		intersect_nodes_    = make(
			[dynamic]IntersectNode(FRAC_DIGITS, U),
			context.temp_allocator,
		) or_return,
		fill_rule_          = fill_rule,
		clip_type_          = clip_type,
		preserve_collinear_ = false,
	}

	pq.init(&ctx.scanline_list_, proc(a, b: fixed_bcd.BCD(FRAC_DIGITS)) -> bool {
			return fixed_bcd.greater(a, b)
		}, pq.default_swap_proc(
			fixed_bcd.BCD(FRAC_DIGITS),
		), allocator = context.temp_allocator) or_return

	AddSubject(&ctx, subjects) or_return
	AddOpenSubject(&ctx, opens) or_return
	AddClip(&ctx, clips) or_return

	slice.stable_sort_by(ctx.minima_list_[:], proc(a, b: ^LocalMinima(FRAC_DIGITS, U)) -> bool {
		if b.vertex.pt.y != a.vertex.pt.y do return fixed_bcd.less(b.vertex.pt.y, a.vertex.pt.y)
		return fixed_bcd.greater(b.vertex.pt.x, a.vertex.pt.x)
	})

	#reverse for minima in ctx.minima_list_ {
		InsertScanline(&ctx, minima.vertex.pt.y) or_return
	}

	y: fixed_bcd.BCD(FRAC_DIGITS)
	pop_ok := PopScanline(&ctx, &y) or_return

	if (!pop_ok) do return nil, nil, nil, nil, .FAILED
	for {
		InsertLocalMinimaIntoAEL(&ctx, y) or_return
		e: ^Active(FRAC_DIGITS, U)
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
	solution_close := make(
		[dynamic][dynamic][2]fixed_bcd.BCD(FRAC_DIGITS),
		context.temp_allocator,
	) or_return
	solution_open := make(
		[dynamic][dynamic][2]fixed_bcd.BCD(FRAC_DIGITS),
		context.temp_allocator,
	) or_return
	non_zero_reserve(&solution_close, len(ctx.outrec_list_)) or_return
	non_zero_reserve(&solution_open, len(ctx.outrec_list_)) or_return

	for out in ctx.outrec_list_ {
		if out.pts == nil do continue

		if out.is_open {
			BuildPath64(out.pts, ctx.reverse_solution_, true, &solution_open) or_return
		} else {
			CleanCollinear(&ctx, out) or_return
			BuildPath64(out.pts, ctx.reverse_solution_, false, &solution_close) or_return
		}
	}

	if len(solution_close) > 0 {
		res = utils_private.make_non_zeroed_slice(
			[][][2]fixed_bcd.BCD(FRAC_DIGITS),
			len(solution_close),
			allocator,
		) or_return
		for i in 0 ..< len(solution_close) {
			res[i] = utils_private.make_non_zeroed_slice(
				[][2]fixed_bcd.BCD(FRAC_DIGITS),
				len(solution_close[i]),
				allocator,
			) or_return

			intrinsics.mem_copy_non_overlapping(
				raw_data(res[i]),
				raw_data(solution_close[i]),
				len(solution_close[i]) * size_of([2]fixed_bcd.BCD(FRAC_DIGITS)),
			)
		}
	}
	if len(solution_open) > 0 {
		res_open = utils_private.make_non_zeroed_slice(
			[][][2]fixed_bcd.BCD(FRAC_DIGITS),
			len(solution_open),
			allocator,
		) or_return

		for i in 0 ..< len(solution_open) {
			res_open[i] = utils_private.make_non_zeroed_slice(
				[][2]fixed_bcd.BCD(FRAC_DIGITS),
				len(solution_open[i]),
				allocator,
			) or_return

			intrinsics.mem_copy_non_overlapping(
				raw_data(res_open[i]),
				raw_data(solution_open[i]),
				len(solution_open[i]) * size_of([2]fixed_bcd.BCD(FRAC_DIGITS)),
			)
		}
	}

	return
}

@(private = "file")
ToBcdPaths :: proc(
	$FRAC: int,
	paths: [][][2]$T,
) -> (
	res: [][][2]fixed_bcd.BCD(FRAC),
	err: Clipper_Error,
) where intrinsics.type_is_float(T) {
	if paths == nil do return nil, nil
	out := make([dynamic][][2]fixed_bcd.BCD(FRAC), context.temp_allocator) or_return
	non_zero_resize(&out, len(paths)) or_return
	for i in 0 ..< len(paths) {
		p := paths[i]
		p_out := make([dynamic][2]fixed_bcd.BCD(FRAC), context.temp_allocator) or_return
		non_zero_resize(&p_out, len(p)) or_return
		for j in 0 ..< len(p) {
			p_out[j] = [2]fixed_bcd.BCD(FRAC) {
				fixed_bcd.from_f64(FRAC, f64(p[j].x)),
				fixed_bcd.from_f64(FRAC, f64(p[j].y)),
			}
		}
		out[i] = p_out[:]
	}
	return out[:], nil
}

@(private = "file")
FromBcdPaths :: proc(
	$T: typeid,
	paths: [][][2]fixed_bcd.BCD($FRAC_DIGITS),
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
			out[i][j] = [2]T{T(fixed_bcd.to_f64(p[j].x)), T(fixed_bcd.to_f64(p[j].y))}
		}
	}
	return out, nil
}

BooleanOp_Fixed :: proc(
	clip_type: ClipType,
	$FRAC_DIGITS: int,
	subjects: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
	clips: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
	opens: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
	fill_rule: FillRule = .Positive,
	allocator := context.allocator,
) -> (
	res: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
	res_open: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
	err: Clipper_Error,
) {
	return BooleanOp_Impl(clip_type, FRAC_DIGITS, subjects, clips, opens, fill_rule, allocator)
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
	FRAC :: fixed_bcd.MAX_FRAC_DIGITS
	sub_bcd := ToBcdPaths(FRAC, subjects) or_return
	clip_bcd := ToBcdPaths(FRAC, clips) or_return
	open_bcd := ToBcdPaths(FRAC, opens) or_return
	res_bcd, res_open_bcd := BooleanOp_Impl(
		clip_type,
		FRAC,
		sub_bcd,
		clip_bcd,
		open_bcd,
		fill_rule,
		allocator = context.temp_allocator,
	) or_return
	res = FromBcdPaths(T, res_bcd, allocator) or_return
	res_open = FromBcdPaths(T, res_open_bcd, allocator) or_return
	return
}

BooleanOpCustomData :: proc(
	clip_type: ClipType,
	$T: typeid,
	subjects: [][][2]T,
	clips: [][][2]T,
	opens: [][][2]T,
	$U: typeid,
	extra_subjects: [][]U,
	extra_clips: [][]U,
	extra_opens: [][]U,
	fill_rule: FillRule = .Positive,
	allocator := context.allocator,
) -> (
	res: [][][2]T,
	res_open: [][][2]T,
	res_extra: [][]U,
	res_open_extra: [][]U,
	err: Clipper_Error,
) where intrinsics.type_is_float(T) {
	FRAC :: fixed_bcd.MAX_FRAC_DIGITS
	sub_bcd := ToBcdPaths(FRAC, subjects) or_return
	clip_bcd := ToBcdPaths(FRAC, clips) or_return
	open_bcd := ToBcdPaths(FRAC, opens) or_return
	res_bcd, res_open_bcd, res_extra, res_open_extra = BooleanOpCustomData_Impl(
		clip_type,
		T,
		sub_bcd,
		clip_bcd,
		open_bcd,
		U,
		extra_subjects,
		extra_clips,
		extra_opens,
		fill_rule,
		allocator,
	) or_return
	res = FromBcdPaths(T, res_bcd, allocator) or_return
	res_open = FromBcdPaths(T, res_open_bcd, allocator) or_return
	return
}

BooleanOpCustomData_Fixed :: proc(
	clip_type: ClipType,
	$FRAC_DIGITS: int,
	subjects: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
	clips: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
	opens: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
	$U: typeid,
	extra_subjects: [][]U,
	extra_clips: [][]U,
	extra_opens: [][]U,
	fill_rule: FillRule = .Positive,
	allocator := context.allocator,
) -> (
	res: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
	res_open: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
	res_extra: [][]U,
	res_open_extra: [][]U,
	err: Clipper_Error,
) {
	return BooleanOpCustomData_Impl(
		clip_type,
		FRAC_DIGITS,
		subjects,
		clips,
		opens,
		U,
		extra_subjects,
		extra_clips,
		extra_opens,
		fill_rule,
		allocator,
	)
}
