// This file is part of the Luau programming language and is licensed under MIT License; see LICENSE.txt for details
#include "lualib.h"

#include "lcommon.h"
#include "lnumutils.h"

#include <math.h>

static void v_rotate(const float* v, const float* q, float* out)
{
    float u[3] = { q[0], q[1], q[2] };
    float c[3] = { q[1] * v[2] - q[2] * v[1], q[2] * v[0] - q[0] * v[2], q[0] * v[1] - q[1] * v[0] };

    float uu = u[0] * u[0] + u[1] * u[1] + u[2] * u[2];
    float uv = u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
    float s = q[3] * q[3] - uu;

    out[0] = v[0] * s + u[0] * 2.f * uv + c[0] * 2.f * q[3];
    out[1] = v[1] * s + u[1] * 2.f * uv + c[1] * 2.f * q[3];
    out[2] = v[2] * s + u[2] * 2.f * uv + c[2] * 2.f * q[3];
}

static int vector_pack(lua_State* L)
{
    int count = lua_gettop(L);
    count = count < 4 ? count : 4;

    double v[4] = { 0 };

    for (int i = 0; i < count; i++) {
      v[i] = luaL_checknumber(L, i + 1);
    }

#if LUA_VECTOR_SIZE == 4
    lua_pushvector(L, float(v[0]), float(v[1]), float(v[2]), float(v[3]));
#else
    lua_pushvector(L, float(v[0]), float(v[1]), float(v[2]));
#endif

    return 1;
}

static int vector_unpack(lua_State* L)
{
    const float* v = luaL_checkvector(L, 1);

    lua_pushnumber(L, v[0]);
    lua_pushnumber(L, v[1]);
    lua_pushnumber(L, v[2]);
#if LUA_VECTOR_SIZE == 4
    lua_pushnumber(L, v[3]);
    return 4;
#else
    return 3;
#endif
}

static int vector_length(lua_State* L)
{
    const float* v = luaL_checkvector(L, 1);

#if LUA_VECTOR_SIZE == 4
    lua_pushnumber(L, sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3]));
#else
    lua_pushnumber(L, sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]));
#endif

    return 1;
}

static int vector_normalize(lua_State* L)
{
    const float* v = luaL_checkvector(L, 1);

#if LUA_VECTOR_SIZE == 4
    float invSqrt = 1.0f / sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3]);

    lua_pushvector(L, v[0] * invSqrt, v[1] * invSqrt, v[2] * invSqrt, v[3] * invSqrt);
#else
    float invSqrt = 1.0f / sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

    lua_pushvector(L, v[0] * invSqrt, v[1] * invSqrt, v[2] * invSqrt);
#endif

    return 1;
}

static int vector_distance(lua_State* L)
{
    const float* a = luaL_checkvector(L, 1);
    const float* b = luaL_checkvector(L, 2);

    float dx = a[0] - b[0];
    float dy = a[1] - b[1];
    float dz = a[2] - b[2];

#if LUA_VECTOR_SIZE == 4
    float dw = a[3] - b[3];
    lua_pushnumber(L, sqrtf(dx * dx + dy * dy + dz * dz + dw * dw));
#else
    lua_pushnumber(L, sqrtf(dx * dx + dy * dy + dz * dz));
#endif

    return 1;
}

static int vector_cross(lua_State* L)
{
    const float* a = luaL_checkvector(L, 1);
    const float* b = luaL_checkvector(L, 2);

#if LUA_VECTOR_SIZE == 4
    lua_pushvector(L, a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0], 0.0f);
#else
    lua_pushvector(L, a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
#endif

    return 1;
}

static int vector_dot(lua_State* L)
{
    const float* a = luaL_checkvector(L, 1);
    const float* b = luaL_checkvector(L, 2);

#if LUA_VECTOR_SIZE == 4
    lua_pushnumber(L, a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
#else
    lua_pushnumber(L, a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
#endif

    return 1;
}

static int vector_angle(lua_State* L)
{
    const float* a = luaL_checkvector(L, 1);
    const float* b = luaL_checkvector(L, 2);
    const float* axis = luaL_optvector(L, 3, nullptr);

    // cross(a, b)
    float cross[] = {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};

    double sinA = sqrt(cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]);
    double cosA = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    double angle = atan2(sinA, cosA);

    if (axis)
    {
        if (cross[0] * axis[0] + cross[1] * axis[1] + cross[2] * axis[2] < 0.0f)
            angle = -angle;
    }

    lua_pushnumber(L, angle);
    return 1;
}

static int vector_floor(lua_State* L)
{
    const float* v = luaL_checkvector(L, 1);

#if LUA_VECTOR_SIZE == 4
    lua_pushvector(L, floorf(v[0]), floorf(v[1]), floorf(v[2]), floorf(v[3]));
#else
    lua_pushvector(L, floorf(v[0]), floorf(v[1]), floorf(v[2]));
#endif

    return 1;
}

static int vector_ceil(lua_State* L)
{
    const float* v = luaL_checkvector(L, 1);

#if LUA_VECTOR_SIZE == 4
    lua_pushvector(L, ceilf(v[0]), ceilf(v[1]), ceilf(v[2]), ceilf(v[3]));
#else
    lua_pushvector(L, ceilf(v[0]), ceilf(v[1]), ceilf(v[2]));
#endif

    return 1;
}

static int vector_abs(lua_State* L)
{
    const float* v = luaL_checkvector(L, 1);

#if LUA_VECTOR_SIZE == 4
    lua_pushvector(L, fabsf(v[0]), fabsf(v[1]), fabsf(v[2]), fabsf(v[3]));
#else
    lua_pushvector(L, fabsf(v[0]), fabsf(v[1]), fabsf(v[2]));
#endif

    return 1;
}

static int vector_sign(lua_State* L)
{
    const float* v = luaL_checkvector(L, 1);

#if LUA_VECTOR_SIZE == 4
    lua_pushvector(L, luaui_signf(v[0]), luaui_signf(v[1]), luaui_signf(v[2]), luaui_signf(v[3]));
#else
    lua_pushvector(L, luaui_signf(v[0]), luaui_signf(v[1]), luaui_signf(v[2]));
#endif

    return 1;
}

static int vector_clamp(lua_State* L)
{
    const float* v = luaL_checkvector(L, 1);
    const float* min = luaL_checkvector(L, 2);
    const float* max = luaL_checkvector(L, 3);

    luaL_argcheck(L, min[0] <= max[0], 3, "max.x must be greater than or equal to min.x");
    luaL_argcheck(L, min[1] <= max[1], 3, "max.y must be greater than or equal to min.y");
    luaL_argcheck(L, min[2] <= max[2], 3, "max.z must be greater than or equal to min.z");

#if LUA_VECTOR_SIZE == 4
    lua_pushvector(
        L,
        luaui_clampf(v[0], min[0], max[0]),
        luaui_clampf(v[1], min[1], max[1]),
        luaui_clampf(v[2], min[2], max[2]),
        luaui_clampf(v[3], min[3], max[3])
    );
#else
    lua_pushvector(L, luaui_clampf(v[0], min[0], max[0]), luaui_clampf(v[1], min[1], max[1]), luaui_clampf(v[2], min[2], max[2]));
#endif

    return 1;
}

static int vector_min(lua_State* L)
{
    int n = lua_gettop(L);
    const float* v = luaL_checkvector(L, 1);

#if LUA_VECTOR_SIZE == 4
    float result[] = {v[0], v[1], v[2], v[3]};
#else
    float result[] = {v[0], v[1], v[2]};
#endif

    for (int i = 2; i <= n; i++)
    {
        const float* b = luaL_checkvector(L, i);

        if (b[0] < result[0])
            result[0] = b[0];
        if (b[1] < result[1])
            result[1] = b[1];
        if (b[2] < result[2])
            result[2] = b[2];
#if LUA_VECTOR_SIZE == 4
        if (b[3] < result[3])
            result[3] = b[3];
#endif
    }

#if LUA_VECTOR_SIZE == 4
    lua_pushvector(L, result[0], result[1], result[2], result[3]);
#else
    lua_pushvector(L, result[0], result[1], result[2]);
#endif

    return 1;
}

static int vector_max(lua_State* L)
{
    int n = lua_gettop(L);
    const float* v = luaL_checkvector(L, 1);

#if LUA_VECTOR_SIZE == 4
    float result[] = {v[0], v[1], v[2], v[3]};
#else
    float result[] = {v[0], v[1], v[2]};
#endif

    for (int i = 2; i <= n; i++)
    {
        const float* b = luaL_checkvector(L, i);

        if (b[0] > result[0])
            result[0] = b[0];
        if (b[1] > result[1])
            result[1] = b[1];
        if (b[2] > result[2])
            result[2] = b[2];
#if LUA_VECTOR_SIZE == 4
        if (b[3] > result[3])
            result[3] = b[3];
#endif
    }

#if LUA_VECTOR_SIZE == 4
    lua_pushvector(L, result[0], result[1], result[2], result[3]);
#else
    lua_pushvector(L, result[0], result[1], result[2]);
#endif

    return 1;
}

static int vector_lerp(lua_State* L)
{
    const float* a = luaL_checkvector(L, 1);
    const float* b = luaL_checkvector(L, 2);
    float t = luaL_checknumber(L, 3);

    float x = a[0] + (b[0] - a[0]) * t;
    float y = a[1] + (b[1] - a[1]) * t;
    float z = a[2] + (b[2] - a[2]) * t;

#if LUA_VECTOR_SIZE == 4
    float w = a[3] + (b[3] - a[3]) * t;
    lua_pushvector(L, x, y, z, w);
#else
    lua_pushvector(L, x, y, z);
#endif

    return 1;
}

#if LUA_VECTOR_SIZE == 4

static int vector_rotate(lua_State* L)
{
    const float* v = luaL_checkvector(L, 1);
    const float* q = luaL_checkvector(L, 2);

    float u[3];
    v_rotate(v, q, u);

#if LUA_VECTOR_SIZE == 4
    lua_pushvector(L, u[0], u[1], u[2], 0.f);
#else
    lua_pushvector(L, u[0], u[1], u[2]);
#endif

    return 1;
}

static int vector_angleaxis(lua_State* L)
{
    float angle = luaL_checknumber(L, 1);
    float ax = luaL_checknumber(L, 2); // TODO vector axis
    float ay = luaL_checknumber(L, 3);
    float az = luaL_checknumber(L, 4);

    float s = sinf(angle * .5f);
    float c = cosf(angle * .5f);
    float length = sqrtf(ax * ax + ay * ay + az * az);
    if (length > 0.f) s /= length;

    lua_pushvector(L, s * ax, s * ay, s * az, c);
    return 1;
}

static int vector_toangleaxis(lua_State* L)
{
    const float* q = luaL_checkvector(L, 1);
    float s = sqrtf(1.f - q[3] * q[3]);
    s = s < .0001f ? 1.f : 1.f / s;
    lua_pushnumber(L, 2.f * acosf(q[3]));
    lua_pushnumber(L, q[0] * s);
    lua_pushnumber(L, q[1] * s);
    lua_pushnumber(L, q[2] * s);
    return 4;
}

static int vector_euler(lua_State* L)
{
  float x = luaL_checknumber(L, 1);
  float y = luaL_checknumber(L, 2);
  float z = luaL_checknumber(L, 3);

  float cx = cosf(x * .5f);
  float sx = sinf(x * .5f);
  float cy = cosf(y * .5f);
  float sy = sinf(y * .5f);
  float cz = cosf(z * .5f);
  float sz = sinf(z * .5f);

  lua_pushvector(L,
    cy * sx * cz + sy * cx * sz,
    sy * cx * cz - cy * sx * sz,
    cy * cx * sz - sy * sx * cz,
    cy * cx * cz + sy * sx * sz
  );

  return 1;
}

static int vector_toeuler(lua_State* L)
{
  const float* q = luaL_checkvector(L, 1);

  float unit = (q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3]);
  float test = q[0] * q[3] - q[1] * q[2];
  const float eps = 1e-7f;

  float x, y, z;

  if (test > (.5f - eps) * unit) {
    x = (float) M_PI / 2.f;
    y = 2.f * atan2f(q[1], q[0]);
    z = 0.f;
  } else if (test < -(.5f - eps) * unit) {
    x = (float) -M_PI / 2.f;
    y = -2.f * atan2f(q[1], q[0]);
    z = 0.f;
  } else {
    x = asinf(2.f * (q[3] * q[0] - q[1] * q[2]));
    y = atan2f(2.f * q[3] * q[1] + 2.f * q[2] * q[0], 1.f - 2.f * (q[0] * q[0] + q[1] * q[1]));
    z = atan2f(2.f * q[3] * q[2] + 2.f * q[0] * q[1], 1.f - 2.f * (q[2] * q[2] + q[0] * q[0]));
  }

  lua_pushnumber(L, x);
  lua_pushnumber(L, y);
  lua_pushnumber(L, z);
  return 3;
}

static int vector_between(lua_State* L)
{
    const float* a = luaL_checkvector(L, 1);
    const float* b = luaL_checkvector(L, 2);

    float dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

    if (dot > .99999f || dot < -.99999f) {
      lua_pushvector(L, 0.f, 0.f, 0.f, 1.f);
      return 1;
    }

    float x = a[1] * b[2] - a[2] * b[1];
    float y = a[2] * b[0] - a[0] * b[2];
    float z = a[0] * b[1] - a[1] * b[0];
    float w = 1.f + dot;

    lua_pushvector(L, x, y, z, w);

    return 1;
}

static int vector_lookdir(lua_State* L)
{
    const float* dir = luaL_checkvector(L, 1);
    const float* up = luaL_optvector(L, 2, (float[3]) { 0.f, 1.f, 0.f });

    float Z[3] = { dir[0], dir[1], dir[2] };
    float length = sqrtf(Z[0] * Z[0] + Z[1] * Z[1] + Z[2] * Z[2]);

    if (length == 0.f) {
        lua_pushvector(L, 0.f, 0.f, 0.f, 1.f);
        return 1;
    }

    Z[0] /= length;
    Z[1] /= length;
    Z[2] /= length;

    float X[3] = { Z[1] * up[2] - Z[2] * up[1], Z[2] * up[0] - Z[0] * up[2], Z[0] * up[1] - Z[1] * up[0] };
    length = sqrtf(X[0] * X[0] + X[1] * X[1] + X[2] * X[2]);

    if (length == 0.f) {
      if (fabs(Z[0]) < .9f) {
        X[0] = 0.f;
        X[1] = Z[2];
        X[2] = -Z[1];
      } else {
        X[0] = Z[2];
        X[1] = 0.f;
        X[2] = -Z[0];
      }

      length = sqrtf(X[0] * X[0] + X[1] * X[1] + X[2] * X[2]);
    }

    X[0] /= length;
    X[1] /= length;
    X[2] /= length;

    float Y[3] = { X[1] * Z[2] - X[2] * Z[1], X[2] * Z[0] - X[0] * Z[2], X[0] * Z[1] - X[1] * Z[0] };

    float m00 = X[0], m01 = Y[0], m02 = Z[0];
    float m10 = X[1], m11 = Y[1], m12 = Z[1];
    float m20 = X[2], m21 = Y[2], m22 = Z[2];

    if (m22 < 0.f) {
      if (m00 > m11) {
        float t = 1.f + m00 - m11 - m22;
        float s = .5f / sqrtf(t);
        lua_pushvector(L, t * s, (m01 + m10) * s, (m20 + m02) * s, (m12 - m21) * s);
      } else {
        float t = 1.f - m00 + m11 - m22;
        float s = .5f / sqrtf(t);
        lua_pushvector(L, (m01 + m10) * s, t * s, (m12 + m21) * s, (m20 - m02) * s);
      }
    } else {
      if (m00 < -m11) {
        float t = 1.f - m00 - m11 + m22;
        float s = .5f / sqrtf(t);
        lua_pushvector(L, (m20 + m02) * s, (m12 + m21) * s, t * s, (m01 - m10) * s);
      } else {
        float t = 1.f + m00 + m11 + m22;
        float s = .5f / sqrtf(t);
        lua_pushvector(L, (m12 - m21) * s, (m20 - m02) * s, (m01 - m10) * s, t * s);
      }
    }

    return 1;
}

static int vector_compose(lua_State* L)
{
    const float* q = luaL_checkvector(L, 1);
    const float* r = luaL_checkvector(L, 2);

    float x = q[0] * r[3] + q[3] * r[0] + q[1] * r[2] - q[2] * r[1];
    float y = q[1] * r[3] + q[3] * r[1] + q[2] * r[0] - q[0] * r[2];
    float z = q[2] * r[3] + q[3] * r[2] + q[0] * r[1] - q[1] * r[0];
    float w = q[3] * r[3] - q[0] * r[0] - q[1] * r[1] - q[2] * r[2];

    lua_pushvector(L, x, y, z, w);

    return 1;
}

static int vector_conjugate(lua_State* L)
{
    const float* q = luaL_checkvector(L, 1);
    lua_pushvector(L, -q[0], -q[1], -q[2], q[3]);
    return 1;
}

static int vector_slerp(lua_State* L)
{
    const float* q = luaL_checkvector(L, 1);
    const float* r = luaL_checkvector(L, 2);
    float t = luaL_checknumber(L, 3);

    float dot = q[0] * r[0] + q[1] * r[1] + q[2] * r[2] + q[3] * r[3];

    if (fabsf(dot) >= 1.f) {
      lua_settop(L, 1);
      return 1;
    }

    float s = 1.f;

    if (dot < 0.f) {
      dot *= -1.f;
      s = -1.f;
    }

    float halfTheta = acosf(dot);
    float sinHalfTheta = sqrtf(1.f - dot * dot);

    if (fabsf(sinHalfTheta) < .001f) {
      float x = s * q[0] * .5f + r[0] * .5f;
      float y = s * q[1] * .5f + r[1] * .5f;
      float z = s * q[2] * .5f + r[2] * .5f;
      float w = s * q[3] * .5f + r[3] * .5f;
      lua_pushvector(L, x, y, z, w);
      return 1;
    }

    float a = sinf((1.f - t) * halfTheta) / sinHalfTheta;
    float b = sinf(t * halfTheta) / sinHalfTheta;

    float x = s * q[0] * a + r[0] * b;
    float y = s * q[1] * a + r[1] * b;
    float z = s * q[2] * a + r[2] * b;
    float w = s * q[3] * a + r[3] * b;
    lua_pushvector(L, x, y, z, w);
    return 1;
}

static int vector_direction(lua_State* L)
{
    const float* q = luaL_checkvector(L, 1);
    float x = -2.f * q[0] * q[2] - 2.f * q[3] * q[1];
    float y = -2.f * q[1] * q[2] + 2.f * q[3] * q[0];
    float z = -1.f + 2.f * q[0] * q[0] + 2.f * q[1] * q[1];
    lua_pushvector(L, x, y, z, 0.f);
    return 1;
}

#endif

static int vector_call(lua_State* L)
{
    lua_remove(L, 1);
    return vector_pack(L);
}

#if LUA_VECTOR_SIZE == 4
static int quat_call(lua_State* L)
{
    lua_remove(L, 1);
    return vector_angleaxis(L);
}
#endif

static const luaL_Reg vectorlib[] = {
    {"pack", vector_pack},
    {"unpack", vector_unpack},
    {"length", vector_length},
    {"normalize", vector_normalize},
    {"distance", vector_distance},
    {"cross", vector_cross},
    {"dot", vector_dot},
    {"angle", vector_angle},
    {"floor", vector_floor},
    {"ceil", vector_ceil},
    {"abs", vector_abs},
    {"sign", vector_sign},
    {"clamp", vector_clamp},
    {"min", vector_min},
    {"max", vector_max},
    {"lerp", vector_lerp},

#if LUA_VECTOR_SIZE == 4
    {"rotate", vector_rotate},
#endif

    {NULL, NULL},
};

static const luaL_Reg quatlib[] = {
#if LUA_VECTOR_SIZE == 4
    {"pack", vector_pack},
    {"unpack", vector_unpack},
    {"length", vector_length},
    {"normalize", vector_normalize},
    {"angleaxis", vector_angleaxis},
    {"toangleaxis", vector_toangleaxis},
    {"euler", vector_euler},
    {"toeuler", vector_toeuler},
    {"between", vector_between},
    {"lookdir", vector_lookdir},
    {"compose", vector_compose},
    {"conjugate", vector_conjugate},
    {"slerp", vector_slerp},
    {"direction", vector_direction},
#endif

    {NULL, NULL},
};

int luaopen_vector(lua_State* L)
{
    // vector
    luaL_register(L, LUA_VECLIBNAME, vectorlib);

    // vector()
    lua_newtable(L);
    lua_pushcfunction(L, vector_call, nullptr);
    lua_setfield(L, -2, "__call");
    lua_setmetatable(L, -2);

#if LUA_VECTOR_SIZE == 4
    lua_pushvector(L, 0.0f, 0.0f, 0.0f, 0.0f);
    lua_setfield(L, -2, "zero");
    lua_pushvector(L, 1.0f, 1.0f, 1.0f, 1.0f);
    lua_setfield(L, -2, "one");
#else
    lua_pushvector(L, 0.0f, 0.0f, 0.0f);
    lua_setfield(L, -2, "zero");
    lua_pushvector(L, 1.0f, 1.0f, 1.0f);
    lua_setfield(L, -2, "one");
#endif

    // quat
    luaL_register(L, LUA_QUATLIBNAME, quatlib);

    // quat()
    lua_newtable(L);
    lua_pushcfunction(L, quat_call, nullptr);
    lua_setfield(L, -2, "__call");
    lua_setmetatable(L, -2);

    // vector metatable
#if LUA_VECTOR_SIZE == 4
    lua_pushvector(L, 0.0f, 0.0f, 0.0f, 0.0f);
#else
    lua_pushvector(L, 0.0f, 0.0f, 0.0f);
#endif

    lua_newtable(L);
    luaL_register(L, NULL, vectorlib);
    luaL_register(L, NULL, quatlib);

    lua_setmetatable(L, -2); // set vector metatable
    lua_pop(L, 1); // pop dummy vector

    lua_pushvalue(L, -1); // vectormt.__index = vectormt
    lua_setfield(L, -2, "__index");

    return 1;
}
