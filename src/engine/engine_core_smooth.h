// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MUJOCO_SRC_ENGINE_ENGINE_CORE_SMOOTH_H_
#define MUJOCO_SRC_ENGINE_ENGINE_CORE_SMOOTH_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>

#ifdef __cplusplus
extern "C" {
#endif
//-------------------------- position --------------------------------------------------------------

/**
 * @brief 正运动学, forward kinematics
 * 
 * @param [in] m 模型对象
 * @param [in|out] d 模型数据
 */
MJAPI void mj_kinematics(const mjModel* m, mjData* d);

/**
 * @brief 质心运动学
 * 
 * map inertias and motion dofs to global frame centered at CoM
 * 1. 更新各个运动子树的质心
 * 2. 更新各个刚体在世界坐标系下关于运动子树质心的惯性张量
 * 3. 更新在世界坐标系下，各个关节相对于质心的平移和旋转运动
 * 
 * @param [in] m 模型对象
 * @param [in|out] d 模型数据
 */
MJAPI void mj_comPos(const mjModel* m, mjData* d);

// compute camera and light positions and orientations
MJAPI void mj_camlight(const mjModel* m, mjData* d);

// compute flex-related quantities
MJAPI void mj_flex(const mjModel* m, mjData* d);

// compute tendon lengths, velocities and moment arms
MJAPI void mj_tendon(const mjModel* m, mjData* d);

// compute actuator transmission lengths and moments
MJAPI void mj_transmission(const mjModel* m, mjData* d);


//-------------------------- inertia ---------------------------------------------------------------

/**
 * @brief 复合刚体算法 CRB
 * 
 * composite rigid body inertia algorithm
 * 
 * @param [in] m 模型对象
 * @param [in|out] d 模型数据
 */
MJAPI void mj_crb(const mjModel* m, mjData* d);

/**
 * @brief 稀疏惯性矩阵的 LTDL 分解
 * 
 * sparse L'*D*L factorizaton of the inertia matrix M, assumed spd
 * 
 * @param [in] m 模型对象
 * @param [in|out] d 模型数据
 * @param [in] M 稀疏惯性矩阵
 * @param [out] qLD 分解后的下三角和对角矩阵
 * @param [out] qLDiagInv 对角矩阵各元素的倒数
 * @param [out] qLDiagSqrtInv 对角矩阵各元素的倒数的开方
 */
MJAPI void mj_factorI(const mjModel* m, mjData* d,
                      const mjtNum* M, mjtNum* qLD, mjtNum* qLDiagInv,
                      mjtNum* qLDiagSqrtInv);

/**
 * @brief 稀疏惯性矩阵的 LTDL 分解
 * 
 * sparse L'*D*L factorizaton of the inertia matrix M, assumed spd
 * 
 * @param [in] m 模型对象
 * @param [in|out] d 模型数据
 */
MJAPI void mj_factorM(const mjModel* m, mjData* d);

// sparse backsubstitution:  x = inv(L'*D*L)*y
MJAPI void mj_solveLD(const mjModel* m, mjtNum* x, int n,
                      const mjtNum* qLD, const mjtNum* qLDiagInv);

// sparse backsubstitution:  x = inv(L'*D*L)*y, use factorization in d
MJAPI void mj_solveM(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* y, int n);

// sparse backsubstitution for one island:  x = inv(L'*D*L)*x, use factorization in d
MJAPI void mj_solveM_island(const mjModel* m, const mjData* d, mjtNum* x, int island);

// half of sparse backsubstitution:  x = sqrt(inv(D))*inv(L')*y
MJAPI void mj_solveM2(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* y, int n);


//-------------------------- velocity --------------------------------------------------------------

// compute cvel, cdof_dot
MJAPI void mj_comVel(const mjModel* m, mjData* d);

// subtree linear velocity and angular momentum
MJAPI void mj_subtreeVel(const mjModel* m, mjData* d);


//-------------------------- RNE -------------------------------------------------------------------

// RNE: compute M(qpos)*qacc + C(qpos,qvel); flg_acc=0 removes inertial term
MJAPI void mj_rne(const mjModel* m, mjData* d, int flg_acc, mjtNum* result);

// RNE with complete data: compute cacc, cfrc_ext, cfrc_int
MJAPI void mj_rnePostConstraint(const mjModel* m, mjData* d);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_CORE_SMOOTH_H_
