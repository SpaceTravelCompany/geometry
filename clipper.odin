package geometry

import "base:intrinsics"
import "base:runtime"
import "core:math"
import "core:mem"
import "core:slice"
import "shared:utils_private"

import "shared:utils_private/fixed_bcd"


FillRule :: enum u8 {
	EvenOdd,
	NonZero,
	Positive,
	Negative,
}

ClipType :: enum u8 {
	NoClip,
	Intersection,
	Union,
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
	Empty,
	OpenStart,
	OpenEnd,
	LocalMax,
	LocalMin,
}

@(private = "file")
DxCheck :: enum u8 {
	None,
	NegativeInf,
	Inf,
}

@(private = "file")
VertexFlags_Set :: bit_set[VertexFlags]

// Clipper context (clipper.engine)
@(private = "file")
Context :: struct($FRAC_DIGITS: int) {
	bot_y_:              fixed_bcd.BCD(FRAC_DIGITS),
	actives_:            ^Active(FRAC_DIGITS),
	sel_:                ^Active(FRAC_DIGITS),
	minima_list_:        [dynamic]^LocalMinima(FRAC_DIGITS),
	vertex_lists_:       [dynamic]^Vertex(FRAC_DIGITS),
	scanline_list_:      [dynamic]fixed_bcd.BCD(FRAC_DIGITS),
	intersect_nodes_:    [dynamic]IntersectNode(FRAC_DIGITS),
	horz_seg_list_:      [dynamic]HorzSegment(FRAC_DIGITS),
	horz_join_list_:     [dynamic]HorzJoin(FRAC_DIGITS),
	current_locmin_idx_: int,
	preserve_collinear_: bool,
	reverse_solution_:   bool,
	has_open_paths_:     bool,
	succeeded_:          bool,
	minima_list_sorted_: bool,
	using_polytree_:     bool,
	fill_rule_:          FillRule,
	clip_type_:          ClipType,
	out:                 [dynamic][dynamic]fixed_bcd.BCD(FRAC_DIGITS),
	outrec_list_:        [dynamic]^OutRec(FRAC_DIGITS),
}

// Scanline: y-coordinate for sweep, linked list (clipper.engine.cpp)
@(private = "file")
Scanline :: struct($FRAC_DIGITS: int) {
	y:    fixed_bcd.BCD(FRAC_DIGITS),
	next: ^Scanline(FRAC_DIGITS),
}

// Vertex: polygon vertex, circular doubly linked (clipper.engine.h)
@(private = "file")
Vertex :: struct($FRAC_DIGITS: int) {
	pt:    [2]fixed_bcd.BCD(FRAC_DIGITS),
	next:  ^Vertex(FRAC_DIGITS),
	prev:  ^Vertex(FRAC_DIGITS),
	flags: VertexFlags_Set,
}

// LocalMinima: bottom of an edge bound (clipper.engine.h)
@(private = "file")
LocalMinima :: struct($FRAC_DIGITS: int) {
	vertex:   ^Vertex(FRAC_DIGITS),
	polytype: PathType,
	is_open:  bool,
}

// OutRec: path in the clipping solution; AEL edges reference it (clipper.engine.h)
@(private = "file")
OutRec :: struct($FRAC_DIGITS: int) {
	idx:        int,
	owner:      ^OutRec(FRAC_DIGITS),
	front_edge: ^Active(FRAC_DIGITS),
	back_edge:  ^Active(FRAC_DIGITS),
	pts:        ^OutPt(FRAC_DIGITS),
	is_open:    bool,
}

// OutPt: output polygon vertex, circular doubly linked (clipper.engine.h)
@(private = "file")
OutPt :: struct($FRAC_DIGITS: int) {
	pt:     [2]fixed_bcd.BCD(FRAC_DIGITS),
	next:   ^OutPt(FRAC_DIGITS),
	prev:   ^OutPt(FRAC_DIGITS),
	outrec: ^OutRec(FRAC_DIGITS),
	horz:   ^HorzSegment(FRAC_DIGITS),
}

// Active: edge in AEL/SEL (clipper.engine.h)
@(private = "file")
Active :: struct($FRAC_DIGITS: int) {
	bot:           [2]fixed_bcd.BCD(FRAC_DIGITS),
	top:           [2]fixed_bcd.BCD(FRAC_DIGITS),
	curr_x:        fixed_bcd.BCD(FRAC_DIGITS),
	dx:            fixed_bcd.BCD(FRAC_DIGITS),
	outrec:        ^OutRec(FRAC_DIGITS),
	prev_in_ael:   ^Active(FRAC_DIGITS),
	next_in_ael:   ^Active(FRAC_DIGITS),
	prev_in_sel:   ^Active(FRAC_DIGITS),
	next_in_sel:   ^Active(FRAC_DIGITS),
	jump:          ^Active(FRAC_DIGITS),
	vertex_top:    ^Vertex(FRAC_DIGITS),
	local_min:     ^LocalMinima(FRAC_DIGITS),
	wind_dx:       i32, // handle negatives
	wind_cnt:      i32,
	wind_cnt2:     i32,
	is_left_bound: bool,
	join_with:     JoinWith,
	dx_check:      DxCheck,
}

// IntersectNode: intersection point and two edges (clipper.engine.h)
@(private = "file")
IntersectNode :: struct($FRAC_DIGITS: int) {
	pt:    [2]fixed_bcd.BCD(FRAC_DIGITS),
	edge1: ^Active(FRAC_DIGITS),
	edge2: ^Active(FRAC_DIGITS),
}

// HorzSegment: horizontal segment for join processing (clipper.engine.h)
@(private = "file")
HorzSegment :: struct($FRAC_DIGITS: int) {
	left_op:       ^OutPt(FRAC_DIGITS),
	right_op:      ^OutPt(FRAC_DIGITS),
	left_to_right: bool,
}

// HorzJoin: horizontal join pair (clipper.engine.h)
@(private = "file")
HorzJoin :: struct($FRAC_DIGITS: int) {
	op1: ^OutPt(FRAC_DIGITS),
	op2: ^OutPt(FRAC_DIGITS),
}

@(private = "file")
AddPaths :: proc(
	ctx: ^Context($FRAC_DIGITS),
	paths: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
	polytype: PathType,
	is_open: bool,
) -> Geometry_Error {
	if is_open do ctx.has_open_paths_ = true
	else do ctx.minima_list_sorted_ = false
	AddPaths_(ctx, paths, polytype, is_open) or_return
}

@(private = "file")
AddLocMin :: proc(
	ctx: ^Context($FRAC_DIGITS),
	vert: ^Vertex(FRAC_DIGITS),
	polytype: PathType,
	is_open: bool,
) -> Geometry_Error {
	if .LocalMin in vert.flags do return nil //make sure the vertex is added only once ...

	vert.flags += {.LocalMin}
	lm := new(LocalMinima(FRAC_DIGITS), context.temp_allocator) or_return
	lm.vertex = vert
	lm.polytype = polytype
	lm.is_open = is_open
	non_zero_append(&ctx.minima_list_, lm) or_return
}

@(private = "file")
AddPaths_ :: proc(
	ctx: ^Context($FRAC_DIGITS),
	paths: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
	polytype: PathType,
	is_open: bool,
) -> Geometry_Error {
	total_vertices := 0
	for path in paths {
		total_vertices += len(path)
	}
	if total_vertices == 0 do return nil

	all_vertices := (^Vertex(FRAC_DIGITS))(
		runtime.mem_alloc_non_zeroed(
			size_of(Vertex(FRAC_DIGITS)) * total_vertices,
			allocator = context.temp_allocator,
		) or_return,
	)

	non_zero_reserve(&ctx.vertex_lists_, len(ctx.vertex_lists_) + len(paths)) or_return
	v := all_vertices
	for path in paths {
		//for each path create a circular double linked list of vertices
		v0 := v
		curr_v := v
		prev_v: ^Vertex(FRAC_DIGITS) = nil

		if len(path) == 0 do continue

		cnt := 0
		for pt in path {
			if prev_v != nil {
				if prev_v.pt == pt do continue // skip duplicates
				prev_v.next = curr_v
			}
			curr_v.prev = prev_v
			curr_v.pt = pt
			curr_v.flags = .Empty
			prev_v = curr_v
			curr_v = intrinsics.ptr_offset(curr_v, 1)
			cnt += 1
		}
		if prev_v == nil || prev_v.prev == nil do continue
		if !is_open && prev_v.pt == v0.pt do prev_v = prev_v.prev

		prev_v.next = v0
		v0.prev = prev_v
		v = curr_v // get ready for next path
		if cnt < 2 || (cnt == 2 && !is_open) do continue

		// find and assign local minima
		going_up, going_up0: bool
		if is_open {
			curr_v = v0.next
			for curr_v != v0 && curr_v.pt[1] == v0.pt[1] {
				curr_v = curr_v.next
			}

			going_up = fixed_bcd.less_than(curr_v.pt[1], v0.pt[1])
			if going_up {
				v0.flags = {.OpenStart}
				AddLocMin(ctx, v0, polytype, true)
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
				AddLocMin(ctx, prev_v, polytype, is_open)
			}
			prev_v = curr_v
			curr_v = curr_v.next
		}

		if is_open {
			prev_v.flags += {.OpenEnd}
			if going_up do prev_v.flags += {.LocalMax}
			else do AddLocMin(ctx, prev_v, polytype, is_open)
		} else if going_up != going_up0 {
			if going_up0 do AddLocMin(ctx, prev_v, polytype, false)
			else do prev_v.flags += {.LocalMax}
		}

		non_zero_append(&ctx.vertex_lists_, v0) or_return
	}
	return nil
}

@(private = "file")
AddSubject :: proc(
	ctx: ^Context($FRAC_DIGITS),
	subjects: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
) -> Geometry_Error {
	AddPaths(ctx, subjects, .Subject, false) or_return
	return nil
}

@(private = "file")
AddClip :: proc(
	ctx: ^Context($FRAC_DIGITS),
	clips: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
) -> Geometry_Error {
	if clips == nil do return nil
	AddPaths(ctx, clips, .Clip, false) or_return
	return nil
}

@(private = "file")
PopScanline :: proc(
	ctx: ^Context($FRAC_DIGITS),
	inout_y: ^fixed_bcd.BCD(FRAC_DIGITS),
) -> (
	bool,
	Geometry_Error,
) {
	if ctx.scanline_list_.len == 0 {
		return false, nil
	}
	inout_y^ = ctx.scanline_list_[len(ctx.scanline_list_) - 1]
	non_zero_resize(&ctx.scanline_list_, len(ctx.scanline_list_) - 1) or_return

	for len(ctx.scanline_list_) != 0 &&
	    fixed_bcd.equal(inout_y^, ctx.scanline_list_[len(ctx.scanline_list_) - 1]) {
		non_zero_resize(&ctx.scanline_list_, len(ctx.scanline_list_) - 1) or_return
	}

	return true, nil
}

@(private = "file")
GetPolyType :: proc "contextless" (e: ^Active($FRAC_DIGITS)) -> PathType {
	return e.local_min.polytype
}

@(private = "file")
IsContributingClosed :: proc "contextless" (
	ctx: ^Context($FRAC_DIGITS),
	e: ^Active(FRAC_DIGITS),
) -> bool {
	fill_rule := ctx.fill_rule_
	clip_type := ctx.clip_type_

	#partial switch fill_rule {
	case .NonZero:
		if abs(e.wind_cnt) != 1 do return false
	case .Positive:
		if e.wind_cnt != 1 do return false
	case .Negative:
		if e.wind_cnt != -1 do return false
	}

	#partial switch clip_type {
	case .Intersection:
		switch fill_rule {
		case .Positive:
			return e.wind_cnt2 > 0
		case .Negative:
			return e.wind_cnt2 == -1
		case:
			return e.wind_cnt2 != 0
		}
	case .Union:
		switch fill_rule {
		case .Positive:
			return e.wind_cnt2 <= 0
		case .Negative:
			return e.wind_cnt2 >= 0
		case:
			return e.wind_cnt2 == 0
		}
	case .Difference:
		result: bool
		switch fill_rule {
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
	case .NoClip:
		return false
	}
	return false // we should never get here
}

@(private = "file")
IsContributingOpen :: proc "contextless" (
	ctx: ^Context($FRAC_DIGITS),
	e: ^Active(FRAC_DIGITS),
) -> bool {
	fill_rule := ctx.fill_rule_
	clip_type := ctx.clip_type_

	is_in_clip: bool
	is_in_subj: bool
	switch fill_rule {
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

	switch clip_type {
	case .Intersection:
		return is_in_clip
	case .Union:
		return !is_in_subj && !is_in_clip
	case:
		return !is_in_clip
	}
}

@(private = "file")
PopLocalMinima :: proc "contextless" (
	ctx: ^Context($FRAC_DIGITS),
	y: fixed_bcd.BCD(FRAC_DIGITS),
) -> ^LocalMinima(FRAC_DIGITS) {
	if ctx.current_locmin_idx_ >= len(ctx.minima_list_) ||
	   !fixed_bcd.equal(y, ctx.minima_list_[ctx.current_locmin_idx_].vertex.pt.y) {
		return nil
	}

	res := ctx.minima_list_[ctx.current_locmin_idx_]
	ctx.current_locmin_idx_ += 1
	return res
}

@(private = "file")
IsOpenActive :: proc "contextless" (e: ^Active($FRAC_DIGITS)) -> bool {
	return e.local_min != nil && e.local_min.is_open
}

@(private = "file")
IsOpenEndVertex :: proc "contextless" (v: ^Vertex($FRAC_DIGITS)) -> bool {
	return .OpenEnd in v.flags || .OpenStart in v.flags
}

@(private = "file")
IsHorizontal :: proc "contextless" (e: ^Active($FRAC_DIGITS)) -> bool {
	return fixed_bcd.equal(e.top.y, e.bot.y)
}

@(private = "file")
GetDx :: proc "contextless" (
	pt1, pt2: [2]fixed_bcd.BCD($F),
) -> (
	fixed_bcd.BCD(FRAC_DIGITS),
	DxCheck,
) {
	dy := fixed_bcd.sub(pt2.y, pt1.y)
	if dy.i != 0 {
		return fixed_bcd.div((fixed_bcd.to_f64(pt2.x) - fixed_bcd.to_f64(pt1.x)), dy), .None
	}
	if dy.i > 0 do return {}, {}, .NegativeInf
	return {}, {}, .Inf
}

@(private = "file")
SetDx :: proc "contextless" (e: ^Active($FRAC_DIGITS)) {
	e.dx, e.dx_check = GetDx(e.bot, e.top)
}

@(private = "file")
IsHeadingRightHorz :: proc "contextless" (e: ^Active($FRAC_DIGITS)) -> bool {
	return e.dx_check == .NegativeInf
}

@(private = "file")
IsHeadingLeftHorz :: proc "contextless" (e: ^Active($FRAC_DIGITS)) -> bool {
	return e.dx_check == .Inf
}

@(private = "file")
InsertLocalMinimaIntoAEL :: proc(
	ctx: ^Context($FRAC_DIGITS),
	bot_y: fixed_bcd.BCD(FRAC_DIGITS),
) -> Geometry_Error {
	for {
		local_minima := PopLocalMinima(ctx, bot_y)
		if local_minima == nil do break

		left_bound: ^Active(FRAC_DIGITS) = nil
		right_bound: ^Active(FRAC_DIGITS) = nil

		if .OpenStart in local_minima.vertex.flags {
			left_bound = nil
		} else {
			left_bound = new_clone(
				Active(FRAC_DIGITS) {
					bot        = local_minima.vertex.pt,
					curr_x     = local_minima.vertex.pt[0],
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
				Active(FRAC_DIGITS) {
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
			} else if left_bound.dx.i < right_bound.dx.i {
				left_bound, right_bound = right_bound, left_bound
			}
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
			InsertRightEdge(ctx, left_bound, right_bound)
			if contributing {
				AddLocalMinPoly(ctx, left_bound, right_bound, left_bound.bot, true) or_return
				if !IsHorizontal(left_bound) do CheckJoinLeft(ctx, left_bound, left_bound.bot) or_return
			}

			for right_bound.next_in_ael != nil &&
			    IsValidAelOrder(ctx, right_bound.next_in_ael, right_bound) {
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
				CheckJoinRight(ctx, right_bound, right_bound.bot)
				InsertScanline(ctx, right_bound.top[1])
			}
		} else if contributing {
			StartOpenPath(ctx, left_bound, left_bound.bot) or_return
		}

		if IsHorizontal(left_bound) {
			PushHorz(ctx, left_bound)
		} else {
			InsertScanline(ctx, left_bound.top[1])
		}
	}
	return nil
}

@(private = "file")
InsertLeftEdge :: proc(ctx: ^Context($FRAC_DIGITS), e: ^Active(FRAC_DIGITS)) {
	if ctx.actives_ == nil {
		e.prev_in_ael = nil
		e.next_in_ael = nil
		ctx.actives_ = e
	} else if !IsValidAelOrder(ctx, ctx.actives_, e) {
		e.prev_in_ael = nil
		e.next_in_ael = ctx.actives_
		ctx.actives_.prev_in_ael = e
		ctx.actives_ = e
	} else {
		e2 := ctx.actives_
		for e2.next_in_ael != nil && IsValidAelOrder(ctx, e2.next_in_ael, e) {
			e2 = e2.next_in_ael
		}

		if e2.join_with == .Right {
			e2 = e2.next_in_ael
		}
		if e2 == nil do return // should never happen and stops compiler warning

		e.next_in_ael = e2.next_in_ael
		if e2.next_in_ael != nil do e2.next_in_ael.prev_in_ael = e
		e.prev_in_ael = e2
		e2.next_in_ael = e
	}
}

@(private = "file")
InsertRightEdge :: proc(
	ctx: ^Context($FRAC_DIGITS),
	left: ^Active(FRAC_DIGITS),
	right: ^Active(FRAC_DIGITS),
) {
	right.next_in_ael = left.next_in_ael
	if left.next_in_ael != nil do left.next_in_ael.prev_in_ael = right
	right.prev_in_ael = left
	left.next_in_ael = right
}

@(private = "file")
SetWindCountForOpenPathEdge :: proc(ctx: ^Context($FRAC_DIGITS), e: ^Active(FRAC_DIGITS)) {
	e2 := ctx.actives_
	if ctx.fill_rule_ == .EvenOdd {
		cnt1, cnt2: int = 0, 0
		for e2 != e {
			if GetPolyType(e2) == .Clip {
				cnt2 += 1
			} else if !IsOpenActive(e2) {
				cnt1 += 1
			}
			e2 = e2.next_in_ael
		}
		e.wind_cnt = (cnt1 & 1) != 0 ? 1 : 0
		e.wind_cnt2 = (cnt2 & 1) != 0 ? 1 : 0
	} else {
		for e2 != e {
			if GetPolyType(e2) == .Clip {
				e.wind_cnt2 += e2.wind_dx
			} else if !IsOpenActive(e2) {
				e.wind_cnt += e2.wind_dx
			}
			e2 = e2.next_in_ael
		}
	}
}

@(private = "file")
SetWindCountForClosedPathEdge :: proc(ctx: ^Context($FRAC_DIGITS), e: ^Active(FRAC_DIGITS)) {
	// Wind counts refer to polygon regions not edges; an edge's WindCnt
	// is the higher of the wind counts for the two regions touching it.
	// (Adjacent regions differ by one. Open paths have no meaningful wind.)
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
		if e2.wind_cnt * e2.wind_dx < 0 {
			// opposite directions so e is outside e2
			if abs(e2.wind_cnt) > 1 {
				if e2.wind_dx * e.wind_dx < 0 {
					e.wind_cnt = e2.wind_cnt
				} else {
					e.wind_cnt = e2.wind_cnt + e.wind_dx
				}
			} else {
				e.wind_cnt = IsOpenActive(e) ? 1 : e.wind_dx
			}
		} else {
			//'e' must be inside 'e2'
			if e2.wind_dx * e.wind_dx < 0 {
				e.wind_cnt = e2.wind_cnt //reversing direction so use the same WC
			} else {
				e.wind_cnt = e2.wind_cnt + e.wind_dx //otherwise keep 'increasing' the WC by 1 (away from 0) ...
			}
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
AddLocalMinPoly :: proc(
	ctx: ^Context($FRAC_DIGITS),
	e1: ^Active(FRAC_DIGITS),
	e2: ^Active(FRAC_DIGITS),
	pt: [2]fixed_bcd.BCD(FRAC_DIGITS),
	is_new: bool,
) -> (
	^OutPt(FRAC_DIGITS),
	Geometry_Error,
) {
	outrec := NewOutRec(ctx) or_return
	e1.outrec = outrec
	e2.outrec = outrec

	if IsOpenActive(e1) {
		outrec.owner = nil
		outrec.is_open = true
		if e1.wind_dx > 0 {
			SetSides(ctx, outrec, e1, e2)
		} else {
			SetSides(ctx, outrec, e2, e1)
		}
	} else {
		prev_hot_edge := GetPrevHotEdge(ctx, e1)
		// e.windDx is the winding direction of the **input** paths
		// and unrelated to the winding direction of output polygons.
		// Output orientation is determined by e.outrec.frontE which is
		// the ascending edge (see AddLocalMinPoly).
		if prev_hot_edge != nil {
			if ctx.using_polytree_ {
				SetOwner(ctx, outrec, prev_hot_edge.outrec)
			}
			if OutrecIsAscending(prev_hot_edge) == is_new {
				SetSides(ctx, outrec, e2, e1)
			} else {
				SetSides(ctx, outrec, e1, e2)
			}
		} else {
			outrec.owner = nil
			if is_new {
				SetSides(ctx, outrec, e1, e2)
			} else {
				SetSides(ctx, outrec, e2, e1)
			}
		}
	}

	op := new_clone(OutPt(FRAC_DIGITS){pt = pt, outrec = outrec}, context.temp_allocator) or_return
	op.next = op
	op.prev = op
	outrec.pts = op
	return op, nil
}

@(private = "file")
AddLocalMaxPoly :: proc(
	ctx: ^Context($FRAC_DIGITS),
	e1: ^Active(FRAC_DIGITS),
	e2: ^Active(FRAC_DIGITS),
	pt: [2]fixed_bcd.BCD(FRAC_DIGITS),
) -> (
	^OutPt(FRAC_DIGITS),
	Geometry_Error,
) {
	if IsJoined(e1) do Split(ctx, e1, pt) or_return
	if IsJoined(e2) do Split(ctx, e2, pt) or_return

	if IsFront(e1) == IsFront(e2) {
		if IsOpenEnd(e1) {
			SwapFrontBackSides(ctx, e1.outrec)
		} else if IsOpenEnd(e2) {
			SwapFrontBackSides(ctx, e2.outrec)
		} else {
			ctx.succeeded_ = false
			return nil, nil
		}
	}

	result := AddOutPt(ctx, e1, pt) or_return
	if e1.outrec == e2.outrec {
		outrec := e1.outrec
		outrec.pts = result
		if ctx.using_polytree_ {
			e := GetPrevHotEdge(ctx, e1)
			if e == nil {
				outrec.owner = nil
			} else {
				SetOwner(ctx, outrec, e.outrec)
			}
			// nb: outrec.owner may not be the real owner; checked in RecursiveCheckOwners()
		}
		UncoupleOutRec(ctx, e1)
		result = outrec.pts
		if outrec.owner != nil && outrec.owner.front_edge == nil {
			outrec.owner = GetRealOutRec(ctx, outrec.owner)
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
JoinOutrecPaths :: proc(
	ctx: ^Context($FRAC_DIGITS),
	e1: ^Active(FRAC_DIGITS),
	e2: ^Active(FRAC_DIGITS),
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
		SetOwner(ctx, e2.outrec, e1.outrec)
	}
	// e1 and e2 are maxima and about to be dropped from the Actives list
	e1.outrec = nil
	e2.outrec = nil
}

@(private = "file")
NewOutRec :: proc(ctx: ^Context($FRAC_DIGITS)) -> (^OutRec(FRAC_DIGITS), Geometry_Error) {
	result := new_clone(
		OutRec(FRAC_DIGITS){idx = len(ctx.outrec_list_)},
		context.temp_allocator,
	) or_return

	non_zero_append(&ctx.outrec_list_, result) or_return
	return result, nil
}

@(private = "file")
AddOutPt :: proc(
	ctx: ^Context($FRAC_DIGITS),
	e: ^Active(FRAC_DIGITS),
	pt: [2]fixed_bcd.BCD(FRAC_DIGITS),
) -> (
	^OutPt(FRAC_DIGITS),
	Geometry_Error,
) {
	// Outrec.pts: circular doubly-linked list; op_back == op_front.next
	outrec := e.outrec
	to_front := IsFront(e)
	op_front := outrec.pts
	op_back := op_front.next

	if to_front {
		if pt == op_front.pt do return op_front, nil
	} else if pt == op_back.pt {
		return op_back, nil
	}

	new_op := new_clone(
		OutPt(FRAC_DIGITS){pt = pt, outrec = outrec, prev = op_front, next = op_back},
		context.temp_allocator,
	) or_return

	op_back.prev = new_op
	op_front.next = new_op
	if to_front do outrec.pts = new_op
	return new_op, nil
}


@(private = "file")
CleanCollinear :: proc(
	ctx: ^Context($FRAC_DIGITS),
	outrec: ^OutRec(FRAC_DIGITS),
) -> Geometry_Error {
	outrec = GetRealOutRec(ctx, outrec)
	if outrec == nil || outrec.is_open do return nil

	if !IsValidClosedPath(outrec.pts) {
		DisposeOutPts(ctx, outrec)
		return nil
	}

	start_op := outrec.pts
	op2 := start_op
	for {
		if IsCollinear(op2.prev.pt, op2.pt, op2.next.pt) &&
		   (op2.pt == op2.prev.pt ||
				   op2.pt == op2.next.pt ||
				   !ctx.preserve_collinear_ ||
				   DotProduct(op2.prev.pt, op2.pt, op2.next.pt) < 0) {

			if op2 == outrec.pts do outrec.pts = op2.prev

			op2 = DisposeOutPt(ctx, op2)

			if !IsValidClosedPath(op2) {
				DisposeOutPts(ctx, outrec)
				return nil
			}

			start_op = op2
			continue
		}
		op2 = op2.next
		if op2 == start_op do break
	}
	FixSelfIntersects(ctx, outrec) or_return
	return nil
}

@(private = "file")
Area_Double :: proc(op: ^OutPt($FRAC_DIGITS)) -> fixed_bcd.BCD(FRAC_DIGITS) {
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

	return result // fixed_bcd.div(result, fixed_bcd.init_const(2, FRAC_DIGITS))
}

@(private = "file")
AreaTriangle :: proc(
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
IsCollinear :: proc(
	pt1: [2]fixed_bcd.BCD($FRAC_DIGITS),
	pt2: [2]fixed_bcd.BCD(FRAC_DIGITS),
	pt3: [2]fixed_bcd.BCD(FRAC_DIGITS),
) -> bool {
	area := AreaTriangle(pt1, pt2, pt3)

	return fixed_bcd.equal(area, fixed_bcd.BCD(FRAC_DIGITS){})
}

@(private = "file")
DisposeOutPts :: proc(ctx: ^Context($FRAC_DIGITS), outrec: ^OutRec(FRAC_DIGITS)) {
	outrec.pts.prev.next = nil
	outrec.pts = nil
	// no need free context.temp_allocator, so skip for - delete
}

@(private = "file")
DoSplitOp :: proc(
	ctx: ^Context($FRAC_DIGITS),
	outrec: ^OutRec(FRAC_DIGITS),
	split_op: ^OutPt(FRAC_DIGITS),
) -> Geometry_Error {
	// split_op.prev->split_op and split_op.next->split_op.next.next intersect
	prev_op := split_op.prev
	next_next_op := split_op.next.next
	outrec.pts = prev_op
	_, ip := LinesIntersect2(prev_op.pt, split_op.pt, split_op.next.pt, next_next_op.pt)

	double_area1 := Area_Double(outrec.pts)
	double_abs_area1 := double_area1
	if double_abs_area1.i < 0 do double_abs_area1.i = -double_abs_area1.i

	if fixed_bcd.less(double_abs_area1, fixed_bcd.init_const(2 * 2, FRAC_DIGITS)) {
		DisposeOutPts(ctx, outrec)
		return nil
	}

	area2 := AreaTriangle(ip, split_op.pt, split_op.next.pt)
	abs_area2 := area2
	if abs_area2.i < 0 do abs_area2.i = -abs_area2.i

	// de-link splitOp and splitOp.next from the path
	// while inserting the intersection point
	if fixed_bcd.equal(ip, prev_op.pt) || fixed_bcd.equal(ip, next_next_op.pt) {
		next_next_op.prev = prev_op
		prev_op.next = next_next_op
	} else {
		new_op2 := new_clone(
			OutPt(FRAC_DIGITS) {
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

	if fixed_bcd.greater_than(abs_area2, fixed_bcd.init_const(1, FRAC_DIGITS)) &&
	   (fixed_bcd.greater(
				   fixed.mul(abs_area2, fixed_bcd.init_const(2, FRAC_DIGITS)),
				   double_abs_area1,
			   ) ||
			   (area2.i > 0) == (double_abs_area1.i > 0)) {
		newOr := NewOutRec(ctx) or_return
		newOr.owner = outrec.owner

		split_op.outrec = newOr
		split_op.next.outrec = newOr
		new_op := new_clone(
			OutPt(FRAC_DIGITS){pt = ip, outrec = newOr, prev = split_op.next, next = split_op},
			context.temp_allocator,
		) or_return

		newOr.pts = new_op
		split_op.prev = new_op
		split_op.next.next = new_op
		// using_polytree_ / splits not used in this Odin OutRec
	} else {
		// delete split_op.next, split_op (caller manages allocator if needed)
	}
	return nil
}

@(private = "file")
FixSelfIntersects :: proc(
	ctx: ^Context($FRAC_DIGITS),
	outrec: ^OutRec(FRAC_DIGITS),
) -> Geometry_Error {
	op2 := outrec.pts
	if op2.prev == op2.next.next do return nil // because triangles can't self-intersect

	for {
		if LinesIntersect3(op2.prev.pt, op2.pt, op2.next.pt, op2.next.next.pt) == .intersect {
			if op2 == outrec.pts || op2.next == outrec.pts do outrec.pts = outrec.pts.prev

			DoSplitOp(ctx, outrec, op2) or_return

			if outrec.pts == nil do break

			op2 = outrec.pts

			if op2.prev == op2.next.next do break
			continue
		} else {
			op2 = op2.next
		}
		if op2 == outrec.pts do break
	}
	return nil
}

@(private = "file")
UpdateOutrecOwner :: proc "contextless" (outrec: ^OutRec(FRAC_DIGITS)) {
	op_curr := outrec.pts
	for {
		op_curr.outrec = outrec
		op_curr = op_curr.next
		if op_curr == outrec.pts do return
	}
}

@(private = "file")
StartOpenPath :: proc(
	ctx: ^Context($FRAC_DIGITS),
	e: ^Active(FRAC_DIGITS),
	pt: [2]fixed_bcd.BCD(FRAC_DIGITS),
) -> (
	^OutPt(FRAC_DIGITS),
	Geometry_Error,
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

	op := new_clone(OutPt(FRAC_DIGITS){pt = pt, outrec = outrec}, context.temp_allocator) or_return
	op.next = op
	op.prev = op
	outrec.pts = op
	return op, nil
}

@(private = "file")
TrimHorz :: proc "contextless" (horz_edge: ^Active(FRAC_DIGITS), preserve_collinear: bool) {
	was_trimmed := false
	pt := NextVertex(horz_edge).pt

	for fixed_bcd.equal(pt[1], horz_edge.top[1]) {
		// always trim 180 deg. spikes (in closed paths)
		// but otherwise break if preserveCollinear = true
		if preserve_collinear &&
		   (fixed_bcd.less(pt[0], horz_edge.top[0]) !=
				   fixed_bcd.less(horz_edge.bot[0], horz_edge.top[0])) {
			break
		}

		horz_edge.vertex_top = NextVertex(horz_edge)
		horz_edge.top = pt
		was_trimmed = true

		if IsMaxima(horz_edge) do break

		pt = NextVertex(horz_edge).pt
	}
	if was_trimmed do SetDx(horz_edge)
}

@(private = "file")
NextVertex :: proc "contextless" (e: ^Active(FRAC_DIGITS)) -> ^Vertex(FRAC_DIGITS) {
	if e.wind_dx > 0 do return e.vertex_top.next
	return e.vertex_top.prev
}

@(private = "file")
IsMaxima :: proc "contextless" (e: ^Active(FRAC_DIGITS)) -> bool {
	return .LocalMax in e.vertex_top.flags
}

@(private = "file")
IsJoined :: proc "contextless" (e: ^Active(FRAC_DIGITS)) -> bool {
	return e.join_with != .NoJoin
}

@(private = "file")
Split :: proc(
	ctx: ^Context($FRAC_DIGITS),
	e: ^Active(FRAC_DIGITS),
	pt: [2]fixed_bcd.BCD(FRAC_DIGITS),
) -> Geometry_Error {
	if e.join_with == .Right {
		e.join_with = .NoJoin
		e.next_in_ael.join_with = .NoJoin
		AddLocalMinPoly(ctx, e, e.next_in_ael, pt, true) or_return
	} else {
		e.join_with = .NoJoin
		e.prev_in_ael.join_with = .NoJoin
		AddLocalMinPoly(ctx, e.prev_in_ael, e, pt, true) or_return
	}
	return nil
}

@(private = "file")
UpdateEdgeIntoAEL :: proc(ctx: ^Context($FRAC_DIGITS), e: ^Active(FRAC_DIGITS)) -> Geometry_Error {
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

	InsertScanline(ctx, e.top.y)
	CheckJoinLeft(ctx, e, e.bot) or_return
	CheckJoinRight(ctx, e, e.bot)
	return nil
}

@(private = "file")
FindEdgeWithMatchingLocMin :: proc "contextless" (
	e: ^Active(FRAC_DIGITS),
) -> ^Active(FRAC_DIGITS) {
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
IsHotEdge :: proc "contextless" (e: ^Active(FRAC_DIGITS)) -> bool {
	return e.outrec != nil
}

@(private = "file")
IsFront :: proc "contextless" (e: ^Active($FRAC_DIGITS)) -> bool {
	return e == e.outrec.front_edge
}

@(private = "file")
IsInvalidPath :: proc "contextless" (op: ^OutPt($FRAC_DIGITS)) -> bool {
	return op == nil || op.next == op
}

@(private = "file")
SetSides :: proc "contextless" (
	ctx: ^Context($FRAC_DIGITS),
	outrec: ^OutRec(FRAC_DIGITS),
	start_edge: ^Active(FRAC_DIGITS),
	end_edge: ^Active(FRAC_DIGITS),
) {
	outrec.front_edge = start_edge
	outrec.back_edge = end_edge
}

@(private = "file")
SwapOutrecs :: proc "contextless" (
	ctx: ^Context($FRAC_DIGITS),
	e1: ^Active(FRAC_DIGITS),
	e2: ^Active(FRAC_DIGITS),
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
	ctx: ^Context($FRAC_DIGITS),
	e1: ^Active(FRAC_DIGITS),
	e2: ^Active(FRAC_DIGITS),
	pt: [2]fixed_bcd.BCD(FRAC_DIGITS),
) -> Geometry_Error {
	// MANAGE OPEN PATH INTERSECTIONS SEPARATELY ...
	if ctx.has_open_paths_ && (IsOpenActive(e1) || IsOpenActive(e2)) {
		if IsOpenActive(e1) && IsOpenActive(e2) do return nil
		edge_o, edge_c: ^Active(FRAC_DIGITS)

		if IsOpenActive(e1) {
			edge_o = e1
			edge_c = e2
		} else {
			edge_o = e2
			edge_c = e1
		}

		if IsJoined(edge_c) do Split(ctx, edge_c, pt) or_return

		if abs(edge_c.wind_cnt) != 1 do return nil

		switch ctx.clip_type_ {
		case .Union:
			if !IsHotEdge(edge_c) do return nil
		case:
			if edge_c.local_min.polytype == .Subject do return nil
		}

		switch ctx.fill_rule_ {
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
				if edge_o.wind_dx > 0 do SetSides(ctx, e3.outrec, edge_o, e3)
				else do SetSides(ctx, e3.outrec, e3, edge_o)

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
	case:
		if ctx.fill_rule_ == .Positive {
			old_e1_windcnt = e1.wind_cnt
			old_e2_windcnt = e2.wind_cnt
		} else {
			old_e1_windcnt = -e1.wind_cnt
			old_e2_windcnt = -e2.wind_cnt
		}
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
			AddLocalMinPoly(ctx, e1, e2, pt, true) or_return
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
		case:
			if ctx.fill_rule_ == .Positive {
				e1_wc2 = e1.wind_cnt2
				e2_wc2 = e2.wind_cnt2
			} else {
				e1_wc2 = -e1.wind_cnt2
				e2_wc2 = -e2.wind_cnt2
			}
		}

		if !IsSamePolyType(e1, e2) {
			AddLocalMinPoly(ctx, e1, e2, pt, false) or_return

		} else if old_e1_windcnt == 1 && old_e2_windcnt == 1 {
			switch ctx.clip_type_ {
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
IsSamePolyType :: proc "contextless" (e1, e2: ^Active(FRAC_DIGITS)) -> bool {
	return e1.local_min.polytype == e2.local_min.polytype
}

@(private = "file")
DeleteFromAEL :: proc "contextless" (ctx: ^Context($FRAC_DIGITS), e: ^Active(FRAC_DIGITS)) {
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
	ctx: ^Context($FRAC_DIGITS),
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
TopX :: proc "contextless" (
	ae: ^Active($FRAC_DIGITS),
	current_y: fixed_bcd.BCD(FRAC_DIGITS),
) -> fixed_bcd.BCD(FRAC_DIGITS) {
	if fixed_bcd.equal(current_y, ae.top.y) || fixed_bcd.equal(ae.top.x, ae.bot.x) do return ae.top.x
	if fixed_bcd.equal(current_y, ae.bot.y) do return ae.bot.x

	return fixed_bcd.add(ae.bot.x, fixed_bcd.mul(ae.dx, fixed_bcd.sub(current_y, ae.bot.y)))
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
	if fixed_bcd.equal(dx, fixed_bcd.BCD(FRAC_DIGITS){}) && fixed_bcd.equal(dy, fixed_bcd.BCD(FRAC_DIGITS){}) do return fixed_bcd.BCD(FRAC_DIGITS){}, fixed_bcd.init_const(1, FRAC_DIGITS)

	den := fixed_bcd.add(fixed_bcd.mul(dx, dx), fixed_bcd.mul(dy, dy))

	num := fixed_bcd.sub(
		fixed_bcd.mul(fixed_bcd.sub(pt.x, line_a.x), dy),
		fixed_bcd.mul(fixed_bcd.sub(pt.y, line_a.y), dx),
	)

	return fixed_bcd.mul(num, num), den
}

@(private = "file")
CheckJoinLeft :: proc(
	ctx: ^Context($FRAC_DIGITS),
	e: ^Active(FRAC_DIGITS),
	pt: [2]fixed_bcd.BCD(FRAC_DIGITS),
	check_curr_x: bool = false,
) -> Geometry_Error {
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

	if (fixed_bcd.less(pt.y, fixed_bcd.add(e.top.y, fixed_bcd.init_const(2, FRAC_DIGITS))) ||
		   fixed_bcd.less(
			   pt.y,
			   fixed_bcd.add(prev.top.y, fixed_bcd.init_const(2, FRAC_DIGITS)),
		   )) &&
	   (fixed_bcd.greater(e.bot.y, pt.y) || fixed_bcd.greater(prev.bot.y, pt.y)) {
		return nil // avoid trivial joins
	}

	if check_curr_x {
		a, b := PerpendicDistFromLineSqrd(pt, prev.bot, prev.top)
		if fixed_bcd.greater(a, fixed_bcd.div(b, fixed_bcd.init_const(4, FRAC_DIGITS))) do return nil // a/b > 0.25, b > 0 always
	} else if !fixed_bcd.equal(e.curr_x, prev.curr_x) {
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
	ctx: ^Context($FRAC_DIGITS),
	e: ^Active(FRAC_DIGITS),
	pt: [2]fixed_bcd.BCD(FRAC_DIGITS),
) {
	_ = ctx
	_ = e
	_ = pt
	// TODO
}

@(private = "file")
IsValidAelOrder :: proc "contextless" (
	ctx: ^Context($FRAC_DIGITS),
	e1: ^Active(FRAC_DIGITS),
	e2: ^Active(FRAC_DIGITS),
) -> bool {
	_ = ctx
	_ = e1
	_ = e2
	// TODO
	return false
}

@(private = "file")
IntersectEdges :: proc(
	ctx: ^Context($FRAC_DIGITS),
	e1: ^Active(FRAC_DIGITS),
	e2: ^Active(FRAC_DIGITS),
	pt: [2]fixed_bcd.BCD(FRAC_DIGITS),
) {
	_ = ctx
	_ = e1
	_ = e2
	_ = pt
	// TODO
}

@(private = "file")
SwapPositionsInAEL :: proc(
	ctx: ^Context($FRAC_DIGITS),
	e1: ^Active(FRAC_DIGITS),
	e2: ^Active(FRAC_DIGITS),
) {
	_ = ctx
	_ = e1
	_ = e2
	// TODO
}

@(private = "file")
PushHorz :: proc(ctx: ^Context($FRAC_DIGITS), e: ^Active(FRAC_DIGITS)) {
	_ = ctx
	_ = e
	// TODO
}

@(private = "file")
InsertScanline :: proc(ctx: ^Context($FRAC_DIGITS), y: fixed_bcd.BCD(FRAC_DIGITS)) {
	append(&ctx.scanline_list_, y)
}

@(private = "file")
StartOpenPath :: proc(
	ctx: ^Context($FRAC_DIGITS),
	e: ^Active(FRAC_DIGITS),
	pt: [2]fixed_bcd.BCD(FRAC_DIGITS),
) {
	_ = ctx
	_ = e
	_ = pt
	// TODO
}

@(private = "file")
PushHorz :: proc(ctx: ^Context($FRAC_DIGITS), e: ^Active(FRAC_DIGITS)) {
	e.next_in_sel = ctx.sel_
	ctx.sel_ = e
}

@(private = "file")
PopHorz :: proc(ctx: ^Context($FRAC_DIGITS), e: ^^Active(FRAC_DIGITS)) -> (bool, Geometry_Error) {
	e^ = ctx.sel_
	if e^ == nil do return false, nil
	ctx.sel_ = ctx.sel_.next_in_sel
	return true, nil
}


@(private = "file")
GetCurrYMaximaVertex_Open :: proc(horz: ^Active($FRAC_DIGITS)) -> ^Vertex(FRAC_DIGITS) {
	return horz.vertex_top
}

@(private = "file")
GetCurrYMaximaVertex :: proc(horz: ^Active($FRAC_DIGITS)) -> ^Vertex(FRAC_DIGITS) {
	return horz.vertex_top
}

@(private = "file")
ResetHorzDirection :: proc(
	horz: ^Active($FRAC_DIGITS),
	vertex_max: ^Vertex(FRAC_DIGITS),
) -> (
	bool,
	fixed_bcd.BCD(FRAC_DIGITS),
	fixed_bcd.BCD(FRAC_DIGITS),
) {
	horz_left := horz.bot.x
	horz_right := horz.top.x
	is_left_to_right := fixed_bcd.less(horz_left, horz_right)
	if !is_left_to_right do horz_left, horz_right = horz_right, horz_left
	return is_left_to_right, horz_left, horz_right
}

@(private = "file")
AddTrialHorzJoin :: proc(ctx: ^Context($FRAC_DIGITS), op: ^OutPt(FRAC_DIGITS)) {
	_ = ctx
	_ = op
	// TODO: add to trial list for later horizontal join processing
}

@(private = "file")
GetLastOp :: proc(horz: ^Active($FRAC_DIGITS)) -> ^OutPt(FRAC_DIGITS) {
	if horz.outrec == nil do return nil
	return horz.outrec.pts
}

// DoHorizontal: Horizontal edges (HEs) at scanline intersections (top or bottom of a scanbeam)
// are processed as if layered. Order doesn't matter. HEs intersect with bottom vertices of
// other HEs[#] and with non-horizontal edges [*]. Once done, intermediate HEs are 'promoted'
// to the next edge in their bounds and may be intersected[%] by other HEs.
// eg: 3 horizontals at a scanline:  /   |     (HE3)o ========%========== o
//           o ======= o(HE2)  /     |         /         /
//       o ============#=========*======*========#=========o (HE1)
@(private = "file")
DoHorizontal :: proc(ctx: ^Context($FRAC_DIGITS), horz: ^Active(FRAC_DIGITS)) -> Geometry_Error {
	horz_is_open := IsOpenActive(horz)
	y := horz.bot.y
	vertex_max: ^Vertex(FRAC_DIGITS)
	if horz_is_open do vertex_max = GetCurrYMaximaVertex_Open(horz)
	else do vertex_max = GetCurrYMaximaVertex(horz)

	is_left_to_right, horz_left, horz_right := ResetHorzDirection(horz, vertex_max)

	if IsHotEdge(horz) {
		pt_bot := [2]fixed_bcd.BCD(FRAC_DIGITS){horz.curr_x, y}
		AddOutPt(ctx, horz, pt_bot) or_return
		AddTrialHorzJoin(ctx, horz.outrec.pts)
	}

	for {
		e: ^Active(FRAC_DIGITS)
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
				if (is_left_to_right && fixed_bcd.greater(e.curr_x, horz_right)) ||
				   (!is_left_to_right && fixed_bcd.less(e.curr_x, horz_left)) {
					break
				}
				if fixed_bcd.equal(e.curr_x, horz.top.x) && !IsHorizontal(e) {
					pt := NextVertex(horz).pt
					if is_left_to_right {
						if IsOpenActive(e) && !IsSamePolyType(e, horz) && !IsHotEdge(e) {
							if fixed_bcd.greater(TopX(e, pt.y), pt.x) do break
						} else if fixed_bcd.greater_than(TopX(e, pt.y), pt.x) do break
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
				CheckJoinRight(ctx, e, pt)
				horz.curr_x = e.curr_x
				e = horz.prev_in_ael
			}

			if horz.outrec != nil do AddTrialHorzJoin(ctx, GetLastOp(horz))
		}

		if horz_is_open && IsOpenEnd(horz) {
			if IsHotEdge(horz) {
				AddOutPt(ctx, horz, horz.top) or_return
				if IsFront(horz) do horz.outrec.front_edge = nil
				else do horz.outrec.back_edge = nil
				horz.outrec = nil
			}
			DeleteFromAEL(ctx, horz)
			return nil
		}
		if !fixed_bcd.equal(NextVertex(horz).pt.y, horz.top.y) do break

		if IsHotEdge(horz) do AddOutPt(ctx, horz, horz.top) or_return
		UpdateEdgeIntoAEL(ctx, horz) or_return
		is_left_to_right, horz_left, horz_right = ResetHorzDirection(horz, vertex_max)
	}

	if IsHotEdge(horz) {
		AddOutPt(ctx, horz, horz.top) or_return
		AddTrialHorzJoin(ctx, horz.outrec.pts)
	}
	UpdateEdgeIntoAEL(ctx, horz) or_return
	return nil
}


@(private = "file")
UpdateHorzSegment :: proc(ctx: ^Context($FRAC_DIGITS), hs: ^HorzSegment(FRAC_DIGITS)) -> bool {
	_ = ctx
	_ = hs
	return false
}

@(private = "file")
DuplicateOp :: proc(
	ctx: ^Context($FRAC_DIGITS),
	op: ^OutPt(FRAC_DIGITS),
	_: bool,
) -> ^OutPt(FRAC_DIGITS) {
	_ = ctx
	return op
}

@(private = "file")
FixOutRecPts :: proc(ctx: ^Context($FRAC_DIGITS), outrec: ^OutRec(FRAC_DIGITS)) {
	_ = ctx
	_ = outrec
}

@(private = "file")
Path2ContainsPath1 :: proc(op1: ^OutPt($FRAC_DIGITS), op2: ^OutPt(FRAC_DIGITS)) -> bool {
	_ = op1
	_ = op2
	return false
}

@(private = "file")
BuildIntersectList :: proc(
	ctx: ^Context($FRAC_DIGITS),
	top_y: fixed_bcd.BCD(FRAC_DIGITS),
) -> bool {
	_ = ctx
	_ = top_y
	return false
}

@(private = "file")
ProcessIntersectList :: proc(ctx: ^Context($FRAC_DIGITS)) {
	_ = ctx
}

@(private = "file")
DoMaxima :: proc(ctx: ^Context($FRAC_DIGITS), e: ^Active(FRAC_DIGITS)) -> ^Active(FRAC_DIGITS) {
	_ = ctx
	return e.next_in_ael
}

@(private = "file")
ConvertHorzSegsToJoins :: proc(ctx: ^Context($FRAC_DIGITS)) {
	j := 0
	for i in 0 ..< len(ctx.horz_seg_list_) {
		if UpdateHorzSegment(ctx, &ctx.horz_seg_list_[i]) do j += 1
	}
	if j < 2 do return
	// TODO: stable_sort by HorzSegSorter; then nested loops to build horz_join_list_
	for i in 0 ..< len(ctx.horz_seg_list_) - 1 {
		hs1 := &ctx.horz_seg_list_[i]
		for k in i + 1 ..< len(ctx.horz_seg_list_) {
			hs2 := &ctx.horz_seg_list_[k]
			if fixed_bcd.greater_than(hs2.left_op.pt.x, hs1.right_op.pt.x) ||
			   hs2.left_to_right == hs1.left_to_right ||
			   fixed_bcd.less_than(hs2.right_op.pt.x, hs1.left_op.pt.x) {
				continue
			}
			curr_y := hs1.left_op.pt.y
			if hs1.left_to_right {
				for fixed_bcd.equal(hs1.left_op.next.pt.y, curr_y) &&
				    fixed_bcd.less_than(hs1.left_op.next.pt.x, hs2.left_op.pt.x) {
					hs1.left_op = hs1.left_op.next
				}
				for fixed_bcd.equal(hs2.left_op.prev.pt.y, curr_y) &&
				    fixed_bcd.less_than(hs2.left_op.prev.pt.x, hs1.left_op.pt.x) {
					hs2.left_op = hs2.left_op.prev
				}
				append(
					&ctx.horz_join_list_,
					HorzJoin(FRAC_DIGITS) {
						op1 = DuplicateOp(ctx, hs1.left_op, true),
						op2 = DuplicateOp(ctx, hs2.left_op, false),
					},
				)
			} else {
				for fixed_bcd.equal(hs1.left_op.prev.pt.y, curr_y) &&
				    fixed_bcd.less_than(hs1.left_op.prev.pt.x, hs2.left_op.pt.x) {
					hs1.left_op = hs1.left_op.prev
				}
				for fixed_bcd.equal(hs2.left_op.next.pt.y, curr_y) &&
				    fixed_bcd.less_than(hs2.left_op.next.pt.x, hs1.left_op.pt.x) {
					hs2.left_op = hs2.left_op.next
				}
				append(
					&ctx.horz_join_list_,
					HorzJoin(FRAC_DIGITS) {
						op1 = DuplicateOp(ctx, hs2.left_op, true),
						op2 = DuplicateOp(ctx, hs1.left_op, false),
					},
				)
			}
		}
	}
}

@(private = "file")
DoIntersections :: proc(ctx: ^Context($FRAC_DIGITS), top_y: fixed_bcd.BCD(FRAC_DIGITS)) {
	if BuildIntersectList(ctx, top_y) {
		ProcessIntersectList(ctx)
		clear(&ctx.intersect_nodes_)
	}
}

@(private = "file")
DoTopOfScanbeam :: proc(ctx: ^Context($FRAC_DIGITS), y: fixed_bcd.BCD(FRAC_DIGITS)) {
	ctx.sel_ = nil // sel_ is reused to flag horizontals (see PushHorz)
	e := ctx.actives_
	for e != nil {
		if fixed_bcd.equal(e.top.y, y) {
			e.curr_x = e.top.x
			if IsMaxima(e) {
				e = DoMaxima(ctx, e) // TOP OF BOUND (MAXIMA)
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
}

@(private = "file")
ProcessHorzJoins :: proc(ctx: ^Context($FRAC_DIGITS)) -> Geometry_Error {
	for i in 0 ..< len(ctx.horz_join_list_) {
		j := &ctx.horz_join_list_[i]
		or1 := GetRealOutRec(ctx, j.op1.outrec)
		or2 := GetRealOutRec(ctx, j.op2.outrec)
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
			FixOutRecPts(ctx, or2)
			if or1.pts.outrec == or2 {
				or1.pts = j.op1
				or1.pts.outrec = or1
			}
			if ctx.using_polytree_ {
				if Path2ContainsPath1(or1.pts, or2.pts) {
					or1.pts, or2.pts = or2.pts, or1.pts
					FixOutRecPts(ctx, or1)
					FixOutRecPts(ctx, or2)
					or2.owner = or1
				} else if Path2ContainsPath1(or2.pts, or1.pts) {
					or2.owner = or1
				} else {
					or2.owner = or1.owner
				}
			} else {
				or2.owner = or1
			}
		} else {
			or2.pts = nil
			if ctx.using_polytree_ do SetOwner(ctx, or2, or1)
			else do or2.owner = or1
		}
	}
	return nil
}

BooleanOp :: proc(
	clip_type: ClipType,
	subjects: [][][2]fixed_bcd.BCD($FRAC_DIGITS),
	clips: [][][2]fixed_bcd.BCD(FRAC_DIGITS) = nil,
	fill_rule: FillRule = .Positive,
	allocator := context.allocator,
) -> (
	res: [][][2]fixed_bcd.BCD(FRAC_DIGITS),
	err: Geometry_Error,
) {
	ctx := Context($FRAC_DIGITS) {
		vertex_lists_ = make([dynamic]^Vertex(FRAC_DIGITS), context.temp_allocator) or_return,
		minima_list_  = make([dynamic]^LocalMinima(FRAC_DIGITS), context.temp_allocator) or_return,
		out           = make(
			[dynamic][dynamic]fixed_bcd.BCD(FRAC_DIGITS),
			context.temp_allocator,
		) or_return,
		fill_rule_    = fill_rule,
		clip_type_    = clip_type,
	}

	AddSubject(ctx, subjects) or_return
	AddClip(ctx, clips) or_return

	y: fixed_bcd.BCD(FRAC_DIGITS)
	pop_ok, pop_err := PopScanline(ctx, &y)
	if pop_err != nil do return res, pop_err
	if !(clip_type == .NoClip || !pop_ok) {
		for ctx.succeeded_ {
			InsertLocalMinimaIntoAEL(ctx, y) or_return
			e: ^Active(FRAC_DIGITS)
			for {
				ok, horz_err := PopHorz(ctx, &e)
				if horz_err != nil do return res, horz_err
				if !ok do break
				DoHorizontal(ctx, e) or_return
			}
			if len(ctx.horz_seg_list_) > 0 {
				ConvertHorzSegsToJoins(ctx)
				clear(&ctx.horz_seg_list_)
			}
			ctx.bot_y_ = y
			pop_ok, pop_err = PopScanline(ctx, &y)
			if pop_err != nil do return res, pop_err
			if !pop_ok do break
			DoIntersections(ctx, y)
			DoTopOfScanbeam(ctx, y)
			for {
				ok, horz_err := PopHorz(ctx, &e)
				if horz_err != nil do return res, horz_err
				if !ok do break
				DoHorizontal(ctx, e) or_return
			}
		}
		if ctx.succeeded_ do ProcessHorzJoins(ctx) or_return
	}

	res = utils_private.make_non_zeroed_slice(
		[]fixed_bcd.BCD(FRAC_DIGITS),
		len(ctx.out),
		allocator,
	) or_return
	defer if err != nil {delete(res, allocator)}

	for o, i in ctx.out {
		defer if err != nil {
			for j in 0 ..< i {
				delete(res[i], allocator)
			}
		}
		res[i] = utils_private.make_non_zeroed_slice(
			[]fixed_bcd.BCD(FRAC_DIGITS),
			len(o),
			allocator,
		) or_return

		mem.copy_non_overlapping(
			raw_data(res[i]),
			raw_data(o),
			len(o) * sizeof(fixed_bcd.BCD(FRAC_DIGITS)),
		)
	}

	return
}

