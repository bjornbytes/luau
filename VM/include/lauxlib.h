#include "lualib.h"

#define LUA_VERSION_NUM 501

#define luaL_checkint(L, i) ((int) luaL_checkinteger(L, i))
#define luaL_optint(L, i, d) ((int) luaL_opinteger(L, i, d))

#define luaL_getsubtable(L, idx, fname) luaL_findtable(L, idx, fname, 1)
