//#############################################################################
//
// FILE:   F28x_Project.h
//
// TITLE:  F28x Project Headerfile and Examples Include File
//
//#############################################################################
//
//
// $Copyright:
// Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

#ifndef F28X_PROJECT_H
#define F28X_PROJECT_H



#include "f28004x_cla_typedefs.h"   // f28004x CLA Type definitions
#include "f28004x_device.h"         // f28004x Headerfile Include File
#include "f28004x_examples.h"       // f28004x Examples Include File

// 添加头文件
// 每新建一个头文件都要添加到此处
// --------------------------
#include "configs.h"                //配置相关头文件
#include "globalvars.h"             //全局变量头文件
#include "board.h"                  //硬件相关头文件

#include "ControlStrategy.h"        //控制函数头文件
#include "spwm.h"                   //spwm头文件
#include "pid.h"                    //pid头文件
#include "PLL.h"                    //PLL头文件
#include "math.h"                   //数学函数相关头文件
#include "SOGI_Forward.h"           //SOGI前向差分离散化头文件
#include "SOGI_Tustin1.h"           //SOGI双线性变换_传递函数离散化头文件
#include "SOGI_Tustin2.h"           //SOGI双线性变换_状态方程离散化头文件
#include "user.h"                   //用户函数头文件


#endif  // end of F28X_PROJECT_H definition

