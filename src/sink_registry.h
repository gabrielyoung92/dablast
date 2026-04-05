/*
 * dablast – side-channel registry that lets streaming code reach the
 * SubchannelSink (SuperframeFilter) created inside DecoderAdapter.
 *
 * Our replacement decoder_adapter.cpp calls dablast_push_sink() right
 * after it creates the SuperframeFilter.  The streaming setup code
 * calls dablast_pop_sink() immediately after addServiceToDecode() returns
 * and registers an UntouchedStreamConsumer on the returned sink.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#pragma once
#include "backend/subchannel_sink.h"

void dablast_push_sink(SubchannelSink* sink);
// Returns nullptr if the queue is empty (should never happen in normal use).
SubchannelSink* dablast_pop_sink();
