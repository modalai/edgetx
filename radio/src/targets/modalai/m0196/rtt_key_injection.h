/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 */

#pragma once

#if defined(HELM_HOST_KEY_INJECTION) && defined(DEBUG_SEGGER_RTT)
void rttKeyInjectionPoll();
#endif
