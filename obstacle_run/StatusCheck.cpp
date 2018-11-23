#include <stdio.h>
#include <unistd.h>

#include "StatusCheck.h"
#include "Head.h"
#include "Action.h"
#include "Walking.h"
#include "MotionStatus.h"
#include "MotionManager.h"
#include "LinuxActionScript.h"

using namespace Robot;

int StatusCheck::m_cur_mode     = READY;
int StatusCheck::m_old_btn      = 0;
int StatusCheck::m_is_started   = 0;

void StatusCheck::Check(CM730 &cm730)
{    
    
    if(m_old_btn == MotionStatus::BUTTON)
    {
        if(m_cur_mode == READY && m_is_started == 0)
        {
            fprintf(stderr, "\n");
			      m_cur_mode = 2;
			      m_old_btn == BTN_START;
        }
    }
    else
    {
        m_old_btn = MotionStatus::BUTTON;
    }
    if(m_old_btn & BTN_START)
    {
        if(m_is_started == 0)
        {
            if(m_cur_mode == START)
            {
                MotionManager::GetInstance()->Reinitialize();
                MotionManager::GetInstance()->SetEnable(true);
                m_is_started = 1;
                fprintf(stderr, "START PROGRAM");
            }
        }
    }
}
