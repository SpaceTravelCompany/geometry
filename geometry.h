#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Slice: same layout as Odin runtime.Raw_Slice (data + len). */
typedef struct Geometry_Slice {
	void*   data;
	int64_t len;
} Geometry_Slice;

/* BCD scaled value: same layout as i128 (lo = low 64 bits, hi = high 64 bits). */
typedef struct Geometry_BCD_Value {
	int64_t lo;
	int64_t hi;
} Geometry_BCD_Value;

typedef struct Geometry_Vec2_BCD {
	Geometry_BCD_Value x;
	Geometry_BCD_Value y;
} Geometry_Vec2_BCD;

typedef struct Geometry_Vertex_BCD {
	Geometry_BCD_Value pos[2];
	float              uvw[3];
	float              color[4];
} Geometry_Vertex_BCD;

typedef struct Geometry_Node_BCD {
	Geometry_Slice     pts;
	Geometry_Slice     curve_pts_ids;
	float              color[4];
	float              stroke_color[4];
	Geometry_BCD_Value thickness;
	_Bool              is_closed;
} Geometry_Node_BCD;

typedef struct Geometry_Shapes_BCD {
	Geometry_Slice nodes;
} Geometry_Shapes_BCD;

typedef struct Geometry_RawShape_BCD {
	Geometry_Slice vertices;
	Geometry_Slice indices;
} Geometry_RawShape_BCD;

/* Float API (legacy): vertex and node use f64 for positions/thickness. */
typedef struct Geometry_Vertex {
	float pos[2];
	float uvw[3];
	float color[4];
} Geometry_Vertex;

typedef struct Geometry_Node {
	Geometry_Slice pts;
	Geometry_Slice curve_pts_ids;
	float          color[4];
	float          stroke_color[4];
	double         thickness;
	_Bool          is_closed;
} Geometry_Node;

typedef struct Geometry_Shapes {
	Geometry_Slice nodes;
} Geometry_Shapes;

typedef struct Geometry_RawShape {
	Geometry_Slice vertices;
	Geometry_Slice indices;
} Geometry_RawShape;

typedef enum Geometry_Error {
	Geometry_Error_None = 0,
	Geometry_Error_IsPointNotLine = 1,
	Geometry_Error_EmptyPolygon = 2,
	Geometry_Error_EmptyColor = 3,
	Geometry_Error_Allocator = 4,
} Geometry_Error;

/* Allocator callbacks (set before using BCD or float compute). */
void geometry_set_alloc_func(void* (*fn)(size_t size));
void geometry_set_calloc_func(void* (*fn)(size_t nmemb, size_t size));
void geometry_set_realloc_func(void* (*fn)(void* old, size_t size));
void geometry_set_free_func(void (*fn)(void* data));

/* BCD value conversion. */
Geometry_BCD_Value geometry_bcd_from_f64(double v);
double             geometry_bcd_to_f64(Geometry_BCD_Value v);

/* BCD polygon API. */
Geometry_Error geometry_shapes_compute_polygon_bcd(
	const Geometry_Shapes_BCD* input,
	Geometry_RawShape_BCD*    out
);
void geometry_raw_shape_free_bcd(Geometry_RawShape_BCD* shape);

/* Float polygon API (legacy). */
Geometry_Error geometry_shapes_compute_polygon(
	const Geometry_Shapes* input,
	Geometry_RawShape*     out
);
void geometry_raw_shape_free(Geometry_RawShape* shape);

#define DEF_FRAC_DIGITS 13

#ifdef __cplusplus
}
#endif

#endif /* GEOMETRY_H */
