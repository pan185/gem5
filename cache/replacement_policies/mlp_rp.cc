/**
 * Copyright (c) 2018 Inria
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "mem/cache/replacement_policies/mlp_rp.hh"

#include <cassert>
#include <memory>

#include "base/logging.hh" // For fatal_if
#include "base/random.hh"
#include "params/MLPRP.hh"
#include "debug/MyRepl.hh" //FIXME: edit
#include "debug/RP.hh"

MLPRP::MLPRP(const Params *p)
    : BaseReplacementPolicy(p),
      numRRPVBits(p->num_bits), hitPriority(p->hit_priority), btp(p->btp)
{
    fatal_if(numRRPVBits <= 0, "There should be at least one bit per RRPV.\n");
}

void
MLPRP::invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
const
{
    std::shared_ptr<MLPReplData> casted_replacement_data =
        std::static_pointer_cast<MLPReplData>(replacement_data);

    // Invalidate entry
    casted_replacement_data->valid = false;
}

void
MLPRP::touch(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    std::shared_ptr<MLPReplData> casted_replacement_data =
        std::static_pointer_cast<MLPReplData>(replacement_data);

    // Update RRPV if not 0 yet
    // Every hit in HP mode makes the entry the last to be evicted, while
    // in FP mode a hit makes the entry less likely to be evicted
    if (hitPriority) {
        casted_replacement_data->rrpv.reset();
    } else {
        casted_replacement_data->rrpv--;
    }
}

// added function
void
MLPRP::touch_with_data(const std::shared_ptr<ReplacementData>& replacement_data, int data) const
{
	DPRINTF(MyRepl,"called touch_with_data: mc=%d\n",data);
	std::shared_ptr<MLPReplData> casted_replacement_data =
		std::static_pointer_cast<MLPReplData>(replacement_data);
	 
	casted_replacement_data->memcost = data;
}

void
MLPRP::reset(const std::shared_ptr<ReplacementData>& replacement_data) const
{
	DPRINTF(MyRepl,"called reset\n");
    std::shared_ptr<MLPReplData> casted_replacement_data =
        std::static_pointer_cast<MLPReplData>(replacement_data);

    // Reset RRPV
    // Replacement data is inserted as "long re-reference" if lower than btp,
    // "distant re-reference" otherwise
    //casted_replacement_data->memcost = 0;

    casted_replacement_data->rrpv.saturate();
    if (random_mt.random<unsigned>(1, 100) <= btp) {
        casted_replacement_data->rrpv--;
    }

    // Mark entry as ready to be used
    casted_replacement_data->valid = true;
}

ReplaceableEntry*
MLPRP::getVictim(const ReplacementCandidates& candidates) const
{
    // There must be at least one replacement candidate
    assert(candidates.size() > 0);

    // Use first candidate as dummy victim
    ReplaceableEntry* victim = candidates[0];

    // Store victim->rrpv in a variable to improve code readability
    int victim_RRPV = std::static_pointer_cast<MLPReplData>(
                        victim->replacementData)->rrpv;

    int victim_MEMCOST = std::static_pointer_cast<MLPReplData>(
		        victim->replacementData)->memcost;

    //FIXME: Zhewen's edit
    for (const auto& candidate : candidates) {
	//candidate.print();
	int candidate_rrpv = std::static_pointer_cast<MLPReplData>(
			candidate->replacementData)->rrpv;
	int candidate_memcost = std::static_pointer_cast<MLPReplData>(
			candidate->replacementData)->memcost;
	bool candidate_valid = std::static_pointer_cast<MLPReplData>(
			candidate->replacementData)->valid;

	DPRINTF(RP,"\tset: %#x way: %#x rrpv: %d memcost: %d (valid:%d)\n", candidate->getSet(), candidate->getWay(), candidate_rrpv, candidate_memcost, candidate_valid);
	DPRINTF(MyRepl,"\tset: %#x way: %#x rrpv: %d memcost: %d (valid:%d)\n", candidate->getSet(), candidate->getWay(), candidate_rrpv, candidate_memcost, candidate_valid);
    }

    // Visit all candidates to find victim
    for (const auto& candidate : candidates) {
        std::shared_ptr<MLPReplData> candidate_repl_data =
            std::static_pointer_cast<MLPReplData>(
                candidate->replacementData);

        // Stop searching for victims if an invalid entry is found
        if (!candidate_repl_data->valid) {
            return candidate;
        }

        // Update victim entry if necessary
        int candidate_RRPV = candidate_repl_data->rrpv;
        int candidate_MEMCOST = candidate_repl_data->memcost;
	if (candidate_RRPV > victim_RRPV) {
            victim = candidate;
            victim_RRPV = candidate_RRPV;
	    victim_MEMCOST = candidate_MEMCOST;
        }
	// break tie
	else if (candidate_RRPV == victim_RRPV) {
	    if (candidate_MEMCOST < victim_MEMCOST) {
		victim = candidate;
		//victim_RRPV = candidate_RRPV;
		victim_MEMCOST = candidate_MEMCOST;
	    }
	}
    }

    // Get difference of victim's RRPV to the highest possible RRPV in
    // order to update the RRPV of all the other entries accordingly
    int diff = std::static_pointer_cast<MLPReplData>(
        victim->replacementData)->rrpv.saturate();

    // No need to update RRPV if there is no difference
    if (diff > 0){
        // Update RRPV of all candidates
        for (const auto& candidate : candidates) {
            std::static_pointer_cast<MLPReplData>(
                candidate->replacementData)->rrpv += diff;
        }
    }

    return victim;
}

std::shared_ptr<ReplacementData>
MLPRP::instantiateEntry()
{
    return std::shared_ptr<ReplacementData>(new MLPReplData(numRRPVBits));
}

MLPRP*
MLPRPParams::create()
{
    return new MLPRP(this);
}
