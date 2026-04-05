package rasterizer

import ".."
import "base:runtime"
import "core:c"
import "core:math/linalg"
import "core:mem"
import pv "engine:clibs/plutovg"
import "engine:utils_private"

__Rasterizer_Error :: enum {
	INVALID_SIZE,
	SURFACE_CREATE_FAILED,
	CANVAS_CREATE_FAILED,
}

Rasterizer_Error :: union #shared_nil {
	__Rasterizer_Error,
	runtime.Allocator_Error,
}

Pixel_Format :: enum u8 {
	// Matches plutovg surface memory layout.
	ARGB8_PREMULTIPLIED,
}

Rasterized_Pixels :: struct {
	width:     int,
	height:    int,
	stride:    int, // bytes per row
	format:    Pixel_Format,
	pixels:    []u8,
	allocator: runtime.Allocator,
}

rasterized_pixels_free :: proc(self: ^Rasterized_Pixels) {
	if self == nil do return
	if self.pixels != nil do delete(self.pixels, self.allocator)
	self^ = {}
}

@(private = "file")
_append_contour_path :: proc(
	canvas: ^pv.plutovg_canvas_t,
	pts: []linalg.Vector2f32,
	curve_flags: []bool,
	is_closed: bool,
) {
	if canvas == nil || len(pts) == 0 do return

	pv.plutovg_canvas_move_to(canvas, pts[0].x, pts[0].y)

	last := len(pts) - 1
	if last <= 0 {
		if is_closed do pv.plutovg_canvas_close_path(canvas)
		return
	}

	i := 0
	for {
		if !is_closed && i >= last do break

		next := i + 1
		if is_closed do next = (i + 1) % len(pts)
		if next >= len(pts) do break

		if next < len(curve_flags) && curve_flags[next] {
			next2 := next + 1
			if is_closed do next2 = (next + 1) % len(pts)
			if next2 >= len(pts) do break

			if next2 < len(curve_flags) && curve_flags[next2] {
				next3 := next2 + 1
				if is_closed do next3 = (next2 + 1) % len(pts)
				if next3 >= len(pts) do break

				pv.plutovg_canvas_cubic_to(
					canvas,
					pts[next].x,
					pts[next].y,
					pts[next2].x,
					pts[next2].y,
					pts[next3].x,
					pts[next3].y,
				)
				i = next3
			} else {
				pv.plutovg_canvas_quad_to(
					canvas,
					pts[next].x,
					pts[next].y,
					pts[next2].x,
					pts[next2].y,
				)
				i = next2
			}
		} else {
			pv.plutovg_canvas_line_to(canvas, pts[next].x, pts[next].y)
			i = next
		}

		if is_closed && i == 0 do break
	}

	if is_closed do pv.plutovg_canvas_close_path(canvas)
}

shapes_to_pixels :: proc(
	src: geometry.shapes,
	width, height: int,
	allocator := context.allocator,
) -> (
	out: Rasterized_Pixels,
	err: Rasterizer_Error = nil,
) {
	if width <= 0 || height <= 0 {
		err = .INVALID_SIZE
		return
	}

	surface := pv.plutovg_surface_create(c.int(width), c.int(height))
	if surface == nil {
		err = .SURFACE_CREATE_FAILED
		return
	}
	defer pv.plutovg_surface_destroy(surface)

	canvas := pv.plutovg_canvas_create(surface)
	if canvas == nil {
		err = .CANVAS_CREATE_FAILED
		return
	}
	defer pv.plutovg_canvas_destroy(canvas)

	clear := pv.plutovg_color_t {
		r = 0.0,
		g = 0.0,
		b = 0.0,
		a = 0.0,
	}
	pv.plutovg_surface_clear(surface, &clear)
	pv.plutovg_canvas_set_fill_rule(canvas, .PLUTOVG_FILL_RULE_NON_ZERO)

	for node in src.nodes {
		if len(node.pts) == 0 do continue

		pv.plutovg_canvas_new_path(canvas)
		for contour, cidx in node.pts {
			curve_flags: []bool = nil
			if node.is_curves != nil && cidx < len(node.is_curves) {
				curve_flags = node.is_curves[cidx]
			}
			_append_contour_path(canvas, contour, curve_flags, node.is_closed)
		}

		has_fill := node.color.w > 0.0
		has_stroke := node.stroke_color.w > 0.0 && node.thickness > 0.0

		if has_fill {
			pv.plutovg_canvas_set_rgba(
				canvas,
				node.color.x,
				node.color.y,
				node.color.z,
				node.color.w,
			)
			if has_stroke {
				pv.plutovg_canvas_fill_preserve(canvas)
			} else {
				pv.plutovg_canvas_fill(canvas)
			}
		}

		if has_stroke {
			pv.plutovg_canvas_set_rgba(
				canvas,
				node.stroke_color.x,
				node.stroke_color.y,
				node.stroke_color.z,
				node.stroke_color.w,
			)
			pv.plutovg_canvas_set_line_width(canvas, node.thickness)
			pv.plutovg_canvas_stroke(canvas)
		}
	}

	stride := int(pv.plutovg_surface_get_stride(surface))
	pixel_bytes := stride * height
	if pixel_bytes < 0 {
		err = .INVALID_SIZE
		return
	}

	pixels := utils_private.make_non_zeroed_slice([]u8, pixel_bytes, allocator) or_return
	mem.copy_non_overlapping(raw_data(pixels), pv.plutovg_surface_get_data(surface), pixel_bytes)

	out = Rasterized_Pixels {
		width     = width,
		height    = height,
		stride    = stride,
		format    = .ARGB8_PREMULTIPLIED,
		pixels    = pixels,
		allocator = allocator,
	}
	return
}
