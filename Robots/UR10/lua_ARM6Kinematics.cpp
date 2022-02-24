#include <lua.hpp>
#include "ARM6Kinematics.h"

/* Copied from lua_unix */
struct def_info {
  const char *name;
  double value;
};

double toolOffsetX = 0.170;
double toolOffsetZ = 0;

void lua_install_constants(lua_State *L, const struct def_info constants[]) {
  int i;
  for (i = 0; constants[i].name; i++) {
    lua_pushstring(L, constants[i].name);
    lua_pushnumber(L, constants[i].value);
    lua_rawset(L, -3);
  }
}


static void lua_pushvector(lua_State *L, std::vector<double> v) {
	int n = v.size();
	lua_createtable(L, n, 0);
	for (int i = 0; i < n; i++) {
		lua_pushnumber(L, v[i]);
		lua_rawseti(L, -2, i+1);
	}
}

static void lua_pushdarray(lua_State *L, double* v, int size) {
	lua_createtable(L, size, 0);
	for (int i = 0; i < size; i++) {
		lua_pushnumber(L, v[i]);
		lua_rawseti(L, -2, i+1);
	}
}


static std::vector<double> lua_checkvector(lua_State *L, int narg) {
	/*
	if (!lua_istable(L, narg))
	luaL_typerror(L, narg, "vector");
	*/
	if ( !lua_istable(L, narg) )
		luaL_argerror(L, narg, "vector");

#if LUA_VERSION_NUM == 502
	int n = lua_rawlen(L, narg);
#else
	int n = lua_objlen(L, narg);
#endif
	std::vector<double> v(n);
	for (int i = 0; i < n; i++) {
		lua_rawgeti(L, narg, i+1);
		v[i] = lua_tonumber(L, -1);
		lua_pop(L, 1);
	}
	return v;
}

static void lua_pushtransform(lua_State *L, Transform t) {
	lua_createtable(L, 4, 0);
	for (int i = 0; i < 4; i++) {
		lua_createtable(L, 4, 0);
		for (int j = 0; j < 4; j++) {
			lua_pushnumber(L, t(i,j));
			lua_rawseti(L, -2, j+1);
		}
		lua_rawseti(L, -2, i+1);
	}
}


static int forward_arm(lua_State *L) {
	std::vector<double> q = lua_checkvector(L, 1);
	Transform t = ARM6_kinematics_forward_arm(&q[0]);
  std::vector<double> pos6D= position6D(t);
  lua_pushvector(L,pos6D);
	return 1;
}

static int forward_wrist(lua_State *L) {
	std::vector<double> q = lua_checkvector(L, 1);
	Transform t = ARM6_kinematics_forward_wrist(&q[0]);
  std::vector<double> pos6D= position6D(t);
  lua_pushvector(L,pos6D);
	return 1;
}


static int forward_arm_qt(lua_State *L) {
  //puma 560 arm webots model FK
	std::vector<double> q = lua_checkvector(L, 1);
	Transform t = ARM6_kinematics_forward_arm(&q[0]);
  std::vector<double> qt=to_quatp(t);

	// lua_pushtransform(L, t);
  lua_pushvector(L,qt);
	return 1;
}

static int inverse_arm(lua_State *L) {
//puma 560 arm webots model IK
	std::vector<double> pArm = lua_checkvector(L, 1);
	std::vector<double> qArmOrg = lua_checkvector(L, 2);
  int wristmode=luaL_optinteger(L,3,0);

	Transform trArm = transform6D(&pArm[0]);
	std::vector<double> qArm1 = ARM6_kinematics_inverse_arm(trArm,&qArmOrg[0],false);
  std::vector<double> qArm2 = ARM6_kinematics_inverse_arm(trArm,&qArmOrg[0],true);

  Transform t1 = ARM6_kinematics_forward_arm(&qArm1[0]);
  Transform t2 = ARM6_kinematics_forward_arm(&qArm2[0]);

  std::vector<double> p1 = position6D(t1);
  std::vector<double> p2 = position6D(t2);



  // printf("Wrist pitch: %.1f / %.1f\n",qArm1[3]*180.0/3.1415, qArm2[3]*180.0/3.1415);

  if (wristmode==0){
    // printf("SP:%.1f vs %.1f  ",qArm1[3]*RAD_TO_DEG, qArm2[3]*RAD_TO_DEG);
    if ((qArm1[3]>-PI/2.0)&&(qArm1[3]<PI*0.75)) lua_pushvector(L, qArm1);
    else lua_pushvector(L, qArm2);
  }else{
    if(wristmode==2){//wrist down
      if (fabs(mod_angle(qArm1[3]))<PI/2.0) lua_pushvector(L, qArm1);
      else lua_pushvector(L, qArm2);
    }else{
      if(wristmode==3){
        if (mod_angle(qArm1[3])>0) lua_pushvector(L, qArm1);
        else    lua_pushvector(L, qArm2);
      }else{         //elbow force FRONT
        if (wristmode==4){
          if ((qArm1[3]>-PI)&&(qArm1[3]<0)) lua_pushvector(L, qArm1);
          else lua_pushvector(L, qArm2);
        }else{
          float total_pitch=qArm1[1]+qArm1[2]+qArm1[3];
          if ((total_pitch>0)&&(total_pitch<PI) )lua_pushvector(L, qArm1);
          else lua_pushvector(L, qArm2);
        }

      }
    }
  }


	return 1;
}

static int inverse_arm_qt(lua_State *L) {
//puma 560 arm webots model IK
	std::vector<double> qtArm = lua_checkvector(L, 1);
	std::vector<double> qArmOrg = lua_checkvector(L, 2);
  int wristmode=luaL_optinteger(L,3,0);

	Transform trArm = transformQuatP(&qtArm[0]);
  std::vector<double> qArm1 = ARM6_kinematics_inverse_arm(trArm,&qArmOrg[0],false);
  std::vector<double> qArm2 = ARM6_kinematics_inverse_arm(trArm,&qArmOrg[0],true);
  // printf("Wrist pitch: %.1f / %.1f\n",qArm1[3]*180.0/3.1415, qArm2[3]*180.0/3.1415);

  if (wristmode==0){
    // printf("SP:%.1f vs %.1f  ",qArm1[3]*RAD_TO_DEG, qArm2[3]*RAD_TO_DEG);
    if ((qArm1[3]>-PI/2.0)&&(qArm1[3]<PI*0.75)) lua_pushvector(L, qArm1);
    else lua_pushvector(L, qArm2);
  }else{
    if(wristmode==2){//wrist down
      if (fabs(mod_angle(qArm1[3]))<PI/2.0) lua_pushvector(L, qArm1);
      else lua_pushvector(L, qArm2);
    }else{
      if(wristmode==3){
        if (mod_angle(qArm1[3])>0) lua_pushvector(L, qArm1);
          else    lua_pushvector(L, qArm2);
      }else{         //elbow force FRONT
        // printf("FORCEFRNT\n");
        if ((qArm1[3]>-PI)&&(qArm1[3]<0)) lua_pushvector(L, qArm1);
        else lua_pushvector(L, qArm2);
      }
    }
  }



  // //return joint angle that is nearest to current wrist pitch angle (no sudden swap)
  // if(
  //   fabs(mod_angle(qArm1[3]-qArmOrg[3])) <
  //   fabs(mod_angle(qArm2[3]-qArmOrg[3])) )  lua_pushvector(L, qArm1);
  // else lua_pushvector(L, qArm2);

	return 1;
}

static int setup_toolparam(lua_State *L) {
	toolOffsetX = lua_tonumber(L, 1);
	toolOffsetZ = lua_tonumber(L, 2);
	return 0;
}

static const struct luaL_Reg kinematics_lib [] = {

  {"setup_tool_param", setup_toolparam},
  {"forward_arm", forward_arm},
  {"forward_wrist", forward_wrist},
  {"forward_arm_qt", forward_arm_qt},
	{"inverse_arm", inverse_arm},
  {"inverse_arm_qt", inverse_arm_qt},
  {NULL, NULL}
};

static const def_info kinematics_constants[] = {
  {NULL, 0}
};

extern "C"
int luaopen_ARM6Kinematics (lua_State *L) {
#if LUA_VERSION_NUM == 502
	luaL_newlib(L, kinematics_lib);
#else
	luaL_register(L, "Kinematics", kinematics_lib);
#endif
	lua_install_constants(L, kinematics_constants);
	return 1;
}
