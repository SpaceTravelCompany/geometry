package svg

import ".."
import "base:runtime"
import "core:encoding/xml"
import "core:math"
import "core:math/linalg"
import "core:mem"
import "core:strconv"
import "core:strings"
import pv "engine:clibs/plutovg"
import "engine:geometry/linalg_ex"
import "engine:utils_private"

__SVG_ERROR :: enum {
	NOT_INITIALIZED,
	INVALID_NODE,
	UNSUPPORTED_FEATURE,
}

SVG_ERROR :: union #shared_nil {
	__SVG_ERROR,
	xml.Error,
	runtime.Allocator_Error,
}

svg_parser :: struct {
	arena_allocator: Maybe(mem.Allocator),
	__arena:         mem.Dynamic_Arena,
	shapes:          geometry.shapes,
}

@(private)
__path_attr :: struct {
	d:              Maybe(string),
	fill:           Maybe(string),
	fill_opacity:   Maybe(f32),
	stroke:         Maybe(string),
	stroke_opacity: Maybe(f32),
	stroke_width:   Maybe(f32),
	transform:      Maybe(string),
}

_EPS: f32 : 0.0001

deinit :: proc(self: ^svg_parser) {
	if self == nil do return
	if self.arena_allocator != nil {
		mem.dynamic_arena_destroy(&self.__arena)
		self.arena_allocator = nil
	}
	self.shapes = {}
}

@(private)
_is_none_value :: proc "contextless" (s: string) -> bool {
	if len(s) != 4 do return false
	return(
		(s[0] == 'n' || s[0] == 'N') &&
		(s[1] == 'o' || s[1] == 'O') &&
		(s[2] == 'n' || s[2] == 'N') &&
		(s[3] == 'e' || s[3] == 'E') \
	)
}


@(private)
_to_geom_point :: proc "contextless" (p: pv.plutovg_point_t) -> linalg.Vector2f32 {
	return linalg.Vector2f32{p.x, -p.y}
}

@(private)
_parse_number_attr :: proc(value: string, out: ^Maybe(f32)) -> SVG_ERROR {
	v := strings.trim_space(value)
	f, ok := strconv.parse_f32(v)
	if !ok do return .INVALID_NODE
	out^ = f
	return nil
}

@(private)
_set_path_attr :: proc(key, value: string, out: ^__path_attr) -> SVG_ERROR {
	switch key {
	case "d":
		out.d = value
	case "fill":
		out.fill = value
	case "fill-opacity":
		_parse_number_attr(value, &out.fill_opacity) or_return
	case "stroke":
		out.stroke = value
	case "stroke-opacity":
		_parse_number_attr(value, &out.stroke_opacity) or_return
	case "stroke-width":
		_parse_number_attr(value, &out.stroke_width) or_return
	case "transform":
		out.transform = value
	}
	return nil
}

@(private)
_parse_style_attr :: proc(style: string, out: ^__path_attr) -> SVG_ERROR {
	i := 0
	for i < len(style) {
		decl_start := i
		for i < len(style) && style[i] != ';' {
			i += 1
		}
		decl := strings.trim_space(style[decl_start:i])
		if i < len(style) && style[i] == ';' do i += 1
		if len(decl) == 0 do continue

		colon := strings.index_byte(decl, ':')
		if colon < 0 do continue

		key := strings.trim_space(decl[:colon])
		value := strings.trim_space(decl[colon + 1:])
		_set_path_attr(key, value, out) or_return
	}
	return nil
}

@(private)
_parse_svg_color :: proc(
	color_text: string,
	opacity: Maybe(f32),
) -> (
	color: linalg.Vector4f32,
	ok: bool,
	err: SVG_ERROR = nil,
) {
	value := strings.trim_space(color_text)
	if len(value) == 0 || _is_none_value(value) {
		return
	}

	clr: pv.plutovg_color_t
	consumed, parse_err := pv.plutovg_color_parse_string(&clr, value)
	if parse_err != nil {
		err = parse_err
		return
	}
	if consumed == 0 {
		err = .INVALID_NODE
		return
	}

	alpha := clr.a
	if o, has := opacity.?; has {
		alpha *= math.clamp(o, 0.0, 1.0)
	}
	color = linalg.Vector4f32{clr.r, clr.g, clr.b, math.clamp(alpha, 0.0, 1.0)}
	ok = color.w > 0.0
	return
}

@(private)
_reset_contour :: proc(
	pts: ^[dynamic]linalg.Vector2f32,
	curves: ^[dynamic]bool,
	arena: mem.Allocator,
) {
	pts^ = make([dynamic]linalg.Vector2f32, arena)
	curves^ = make([dynamic]bool, arena)
}

@(private)
_normalize_contour_winding :: proc(
	pts_out: ^[dynamic][]linalg.Vector2f32,
	curves_out: ^[dynamic][]bool,
	arena: mem.Allocator,
) -> SVG_ERROR {
	if len(pts_out^) != len(curves_out^) do return .INVALID_NODE

	for i in 0 ..< len(pts_out^) {
		pts := pts_out^[i]
		curves := curves_out^[i]
		if len(pts) < 3 do continue
		if len(pts) != len(curves) do return .INVALID_NODE

		is_hole := geometry.IsHoleContour(i, pts_out^[:])
		orientation := linalg_ex.GetPolygonOrientation(pts)
		need_reverse :=
			(!is_hole && orientation != .CounterClockwise) ||
			(is_hole && orientation != .Clockwise)
		if need_reverse {
			err: geometry.shape_error
			pts_out^[i], curves_out^[i], err = geometry.ReverseShapeCloseCurve(pts, curves, arena)
			if err != nil do return .INVALID_NODE
		}
	}
	return nil
}

@(private)
_finalize_contour :: proc(
	inout_pts: ^[dynamic]linalg.Vector2f32,
	inout_curves: ^[dynamic]bool,
	is_closed: bool,
	pts_out: ^[dynamic][]linalg.Vector2f32,
	curves_out: ^[dynamic][]bool,
	arena: mem.Allocator,
) -> SVG_ERROR {
	if len(inout_pts^) == 0 {
		_reset_contour(inout_pts, inout_curves, arena)
		return nil
	}
	if len(inout_pts^) != len(inout_curves^) {
		return .INVALID_NODE
	}

	//if same first and last point of a closed contour, remove the last point to avoid duplicate in geometry.shapes_compute_polygon.
	if is_closed &&
	   len(inout_pts^) > 1 &&
	   !inout_curves^[len(inout_curves^) - 1] &&
	   inout_pts^[len(inout_pts^) - 1] == inout_pts^[0] {
		non_zero_resize_dynamic_array(inout_pts, len(inout_pts^) - 1) or_return
		non_zero_resize_dynamic_array(inout_curves, len(inout_curves^) - 1) or_return
	}

	// geometry.shapes_compute_polygon expects the first point flag to be non-curve.
	// If needed, rotate contour start to the first non-curve point while preserving pairing.
	if is_closed && len(inout_curves^) > 0 && inout_curves^[0] {
		start := -1
		for i in 0 ..< len(inout_curves^) {
			if !inout_curves^[i] {
				start = i
				break
			}
		}
		if start < 0 {
			return .INVALID_NODE
		}
		if start > 0 {
			rot_pts: [dynamic]linalg.Vector2f32 = make([dynamic]linalg.Vector2f32, arena)
			rot_curves: [dynamic]bool = make([dynamic]bool, arena)
			non_zero_reserve(&rot_pts, len(inout_pts^)) or_return
			non_zero_reserve(&rot_curves, len(inout_curves^)) or_return
			for i in 0 ..< len(inout_pts^) {
				src := (start + i) % len(inout_pts^)
				non_zero_append(&rot_pts, inout_pts^[src]) or_return
				non_zero_append(&rot_curves, inout_curves^[src]) or_return
			}
			inout_pts^ = rot_pts
			inout_curves^ = rot_curves
		}
	}

	if is_closed && len(inout_pts^) >= 2 {
		non_zero_append(pts_out, inout_pts^[:]) or_return
		non_zero_append(curves_out, inout_curves^[:]) or_return
	}

	_reset_contour(inout_pts, inout_curves, arena)
	return nil
}

@(private)
_path_to_closed_contours :: proc(
	path: ^pv.plutovg_path_t,
	pts_out: ^[dynamic][]linalg.Vector2f32,
	curves_out: ^[dynamic][]bool,
	arena: mem.Allocator,
) -> SVG_ERROR {
	current_pts: [dynamic]linalg.Vector2f32
	current_curves: [dynamic]bool
	_reset_contour(&current_pts, &current_curves, arena)

	it: pv.plutovg_path_iterator_t
	pv.plutovg_path_iterator_init(&it, path)
	for pv.plutovg_path_iterator_has_next(&it) {
		points: [3]pv.plutovg_point_t
		cmd := pv.plutovg_path_iterator_next(&it, &points)

		switch cmd {
		case .PLUTOVG_PATH_COMMAND_MOVE_TO:
			_finalize_contour(
				&current_pts,
				&current_curves,
				true,
				pts_out,
				curves_out,
				arena,
			) or_return
			non_zero_append(&current_pts, _to_geom_point(points[0])) or_return
			non_zero_append(&current_curves, false) or_return
		case .PLUTOVG_PATH_COMMAND_LINE_TO:
			if len(current_pts) == 0 do return .INVALID_NODE
			non_zero_append(&current_pts, _to_geom_point(points[0])) or_return
			non_zero_append(&current_curves, false) or_return
		case .PLUTOVG_PATH_COMMAND_CUBIC_TO:
			if len(current_pts) == 0 do return .INVALID_NODE
			non_zero_append(
				&current_pts,
				_to_geom_point(points[0]),
				_to_geom_point(points[1]),
				_to_geom_point(points[2]),
			) or_return
			non_zero_append(&current_curves, true, true, false) or_return
		case .PLUTOVG_PATH_COMMAND_CLOSE:
			if len(current_pts) == 0 do continue
			_finalize_contour(
				&current_pts,
				&current_curves,
				true,
				pts_out,
				curves_out,
				arena,
			) or_return
		case:
			return .UNSUPPORTED_FEATURE
		}
	}

	// Trailing subpath can be open in source data, but fill semantics still close it.
	_finalize_contour(&current_pts, &current_curves, true, pts_out, curves_out, arena) or_return
	return nil
}

@(private)
_path_element_to_shape_node :: proc(
	e: ^xml.Element,
	arena: mem.Allocator,
) -> (
	node: geometry.shape_node,
	ok: bool,
	err: SVG_ERROR = nil,
) {
	attr := __path_attr{}
	for a in e.attribs {
		switch a.key {
		case "style":
			_parse_style_attr(a.val, &attr) or_return
		case:
			_set_path_attr(strings.trim_space(a.key), strings.trim_space(a.val), &attr) or_return
		}
	}

	if attr.d == nil do return

	fill_color := linalg.Vector4f32{0.0, 0.0, 0.0, 1.0}
	has_fill := true
	if fill, has := attr.fill.?; has {
		fill_color, has_fill, err = _parse_svg_color(fill, attr.fill_opacity)
		if err != nil do return
	} else if a, has := attr.fill_opacity.?; has {
		fill_color.w = math.clamp(a, 0.0, 1.0)
		has_fill = fill_color.w > 0.0
	}
	if !has_fill do return

	stroke_color := linalg.Vector4f32{}
	has_stroke := false
	if stroke, has := attr.stroke.?; has {
		stroke_color, has_stroke, err = _parse_svg_color(stroke, attr.stroke_opacity)
		if err != nil do return
	}
	stroke_width: f32 = 0.0
	if has_stroke {
		stroke_width = 1.0
		if w, has := attr.stroke_width.?; has {
			stroke_width = math.max(w, 0.0)
		}
	}

	path := pv.plutovg_path_create()
	if path == nil {
		err = .INVALID_NODE
		return
	}
	defer pv.plutovg_path_destroy(path)

	d := strings.trim_space(attr.d.?)
	if len(d) == 0 do return
	d_ok, d_err := pv.plutovg_path_parse_string(path, d)
	if d_err != nil {
		err = d_err
		return
	}
	if !d_ok {
		err = .INVALID_NODE
		return
	}

	if t, has := attr.transform.?; has {
		t = strings.trim_space(t)
		if len(t) > 0 {
			m: pv.plutovg_matrix_t
			t_ok, t_err := pv.plutovg_matrix_parse_string(&m, t)
			if t_err != nil {
				err = t_err
				return
			}
			if !t_ok {
				err = .INVALID_NODE
				return
			}
			pv.plutovg_path_transform(path, &m)
		}
	}

	pts: [dynamic][]linalg.Vector2f32 = make([dynamic][]linalg.Vector2f32, arena)
	curves: [dynamic][]bool = make([dynamic][]bool, arena)
	_path_to_closed_contours(path, &pts, &curves, arena) or_return
	_normalize_contour_winding(&pts, &curves, arena) or_return
	if len(pts) == 0 do return

	node = geometry.shape_node {
		pts          = pts[:],
		is_curves    = curves[:],
		color        = fill_color,
		stroke_color = stroke_color,
		thickness    = stroke_width,
		is_closed    = true,
	}
	ok = true
	return
}

@(private)
_parse_svg_element :: proc(
	xml_doc: ^xml.Document,
	idx: xml.Element_ID,
	nodes: ^[dynamic]geometry.shape_node,
	arena: mem.Allocator,
) -> SVG_ERROR {
	e := &xml_doc.elements[idx]
	if e.ident == "path" {
		node, ok, path_err := _path_element_to_shape_node(e, arena)
		if path_err != nil do return path_err
		if ok {
			non_zero_append(nodes, node) or_return
		}
	}

	for v in e.value {
		switch value in v {
		case xml.Element_ID:
			_parse_svg_element(xml_doc, value, nodes, arena) or_return
		case string:
		}
	}
	return nil
}

init_parse :: proc(
	svg_data: []u8,
	allocator: mem.Allocator = context.allocator,
) -> (
	parser: svg_parser,
	err: SVG_ERROR = nil,
) {
	if len(svg_data) == 0 {
		err = .INVALID_NODE
		return
	}

	parser = {}
	mem.dynamic_arena_init(&parser.__arena, allocator, allocator)
	arena := mem.dynamic_arena_allocator(&parser.__arena)
	parser.arena_allocator = arena
	defer if err != nil {
		deinit(&parser)
	}

	xml_doc, xml_err := xml.parse(svg_data, allocator = context.temp_allocator)
	if xml_err != nil {
		err = xml_err
		return
	}
	defer xml.destroy(xml_doc, context.temp_allocator)

	nodes: [dynamic]geometry.shape_node = make([dynamic]geometry.shape_node, arena)
	_parse_svg_element(xml_doc, 0, &nodes, arena) or_return

	parser.shapes = geometry.shapes {
		nodes = nodes[:],
	}
	return
}
