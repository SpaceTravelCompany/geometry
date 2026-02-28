#ifndef GEOMETRY_H
#define GEOMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct { float x, y; } Geometry_Vec2;
typedef struct { float x, y, z; } Geometry_Vec3;
typedef struct { float x, y, z, w; } Geometry_Vec4;

/* Geometry_Slice matches Odin runtime.Raw_Slice { data, len } - no copy needed */
typedef struct {
    void* data;
    ssize_t len;
} Geometry_Slice;

#pragma pack(push, 1)
typedef struct {
    Geometry_Vec2 pos;
    Geometry_Vec3 uvw;
    Geometry_Vec4 color;
} Geometry_Vertex;
#pragma pack(pop)

typedef struct {
    Geometry_Slice vertices;
    Geometry_Slice indices;
} Geometry_RawShape;

typedef struct {
    Geometry_Slice pts;
    Geometry_Slice curve_pts_ids;
    Geometry_Vec4 color;
    Geometry_Vec4 stroke_color;
    double thickness;
    bool is_closed;
} Geometry_Node;

typedef struct {
    Geometry_Slice nodes;
} Geometry_Shapes;

typedef int64_t Geometry_Fixed;

typedef struct {
    Geometry_Fixed x, y;
} Geometry_Vec2_Fixed;

#pragma pack(push, 1)
typedef struct {
    Geometry_Fixed pos[2];
    Geometry_Vec3 uvw;
    Geometry_Vec4 color;
} Geometry_Vertex_Fixed;
#pragma pack(pop)

typedef struct {
    Geometry_Slice vertices;
    Geometry_Slice indices;
} Geometry_RawShape_Fixed;

typedef struct {
    Geometry_Slice pts;
    Geometry_Slice n_polys;
    Geometry_Slice curve_pts_ids;
    Geometry_Vec4 color;
    Geometry_Vec4 stroke_color;
    Geometry_Fixed thickness;
    bool is_closed;
} Geometry_Node_Fixed;

typedef struct {
    Geometry_Slice nodes;
} Geometry_Shapes_Fixed;

typedef enum {
    Geometry_Error_None = 0,
    Geometry_Error_IsPointNotLine = 1,
    Geometry_Error_EmptyPolygon = 2,
    Geometry_Error_EmptyColor = 3,
    Geometry_Error_Allocator = 4,
} Geometry_Error;

typedef struct {
    void* procedure;
    void* data;
} Odin_Allocator;

void geometry_set_alloc_func(void* (*alloc)(size_t));
void geometry_set_calloc_func(void* (*calloc)(size_t, size_t));
void geometry_set_realloc_func(void* (*realloc)(void*, size_t));
void geometry_set_free_func(void (*free)(void*));

Geometry_Error geometry_shapes_compute_polygon(const Geometry_Shapes* input, Geometry_RawShape* out);
void geometry_raw_shape_free(Geometry_RawShape* shape);

/* Fixed-point helpers */
Geometry_Fixed geometry_fixed_from_f64(double v);
double geometry_fixed_to_f64(Geometry_Fixed v);

Geometry_Error geometry_shapes_compute_polygon_fixed(const Geometry_Shapes_Fixed* input, Geometry_RawShape_Fixed* out);
void geometry_raw_shape_free_fixed(Geometry_RawShape_Fixed* shape);

#ifdef __cplusplus
}
#endif

#endif /* GEOMETRY_H */
