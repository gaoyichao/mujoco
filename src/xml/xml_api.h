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

#ifndef MUJOCO_SRC_XML_XML_API_H_
#define MUJOCO_SRC_XML_XML_API_H_

#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 解析 MJCF 或 URDF 格式的 XML 文件，编译它，并返回底层的模型对象
 * 
 * @param [in] filename 文件名
 * @param [in] vfs 虚拟文件系统对象，如果非 NULL 则优先从 vfs 中查找文件
 * @param [out] error 错误消息缓存
 * @param [in] error_sz 错误消息缓存大小
 * @return 底层的模型数据对象
 */
MJAPI mjModel* mj_loadXML(const char* filename, const mjVFS* vfs, char* error, int error_sz);

// update XML data structures with info from low-level model, save as MJCF
MJAPI int mj_saveLastXML(const char* filename, const mjModel* m, char* error, int error_sz);

// free last XML model if loaded; called internally at each load
MJAPI void mj_freeLastXML(void);

// print internal XML schema as plain text or HTML, with style-padding or &nbsp;
MJAPI int mj_printSchema(const char* filename, char* buffer, int buffer_sz,
                         int flg_html, int flg_pad);


#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_XML_XML_API_H_
