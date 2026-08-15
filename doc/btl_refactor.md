
The point of the refactor:
    - Keep only single counting logic, not per bit-rate
    - Generate only single RX trigger -> Reduce IPT from 2 to 1
    - Achieve simpler design, less logic and better timing!

BTL Inputs:

- Bit time configuration (PROP, PH1, PH2, SJW) - Quasy-static.
- sync_edge - can occur anytime, depends on CAN_RX
- sp_control - bit-rate + sampling
    - changes with PC FSMs RX Trigger
    - sampling should be split from bit-rate, bit-rate wil go to BTL, sampling control to BM
    - should be registered, but now has "prefetch"
- sync_control - type of synchronization
    - changes with PC FSMs RX Trigger
    - registered in PC FSM!
- no_pos_resync - when current CAN_TX is DOMINANT
    - CAN_TX updated at start of bit time
    - positive resynchronization should not take effect then.
- nbt_ctrs_en, dbt_ctrs_en
    - clock enables
    - FWIW, can be even completely removed, the few flops wont make difference!

BTL Outputs:

- RX Trigger
    - Active at sample point
    - Used to sample the RX Data by PC FSM!
    - Should be registered
    - Can't occur due to hard-synchronization since hard-sync forces the edge
      that caused it to lie in SYNC segment. This means that only TX trigger
      can occur due to the HARD SYNC.
    - Can't occur due to "immediate resynchronization with e < SJW" since
      resynchronization in TSEG1 lengthens the TSEG1, not shortens it!
    - Therefore it is well plannable ahead, and we never have to suffer the
      pipeline delay for information that cause occurence of RX Trigger!
    - Can only occur due to:
        - Regular TSEG1 end
        - Resynchronized TSEG1 end due to TSEG1 lengthened by previous edge.

- TX Trigger
    - Active at the end of bit (last cycle of bit)
    - Used to sample the PC FSM
    - Can occur due to:
        - Regular end of TSEG2 - plannable ahead
        - Immediate resynchronization shorter than SJW - edge will cause end
          of TSEG2 immediately, and therefore TX trigger must be set in the
          same cycle -> No time to be registered
        - Hard synchronization - If hard-sync occurs at TSEG2, current bit
          must be immediately ended, and new started. To keep the TX/RX
          triggers alternating, TX Trigger must be generated.
    - Due to the need to be generated "immediately", can't be registered,
      but this does not mind since it is used only to clock the TX Data,
      not to gate anything in PC FSM!

STATE information in BTL:
- Current segment of bit - TSEG1 or TSEG2
    - Held by the FSM.
    - FSM will tick upon the RX / TX trigger!

- Prescaler counter - generates "tq_edge", an enable for the whole machinery
    - Can run all the time when circuit is enabled.
    - Must reset when a segment ends to prevent misalignment between Nominal
      and Data bit rates.
    - Can be upcounting or downcounting
    - Downcounting needs to know the prescaler at the moment when being preloaded
      but has better detection of end (simply compare with zero)
    - Should use downcounting!
- Segment counter
    - Can run all the time when circuit is enabled
    - Must reset when a segment ends -> Preload length of next segment!
    - If made up-counting (as current):
        - The target to count to does not need to be known at segment end
        - The detetion of end is about comparing with expected duration that
          can change due to resynchronization
    - If made down-counting:
        - The target to count needs to be known at sample point or at bit end
            - This should be OK with having only one RX trigger, but if we rework
              the signalling of bit-rate switching from PC FSM to be combinatorial!
              Due to this we currenly need the prefetch on the sp_control!
        - Resynchronization logic is harder because computing phase error is
          harder !
            - TODO: Compute this!
    - We will try downcounting !
- Current bit-rate
    - Now held in PC FSM, but can be moved entirely to BTL and controlled only
      by PC FSM combo outputs active in RX trigger (for switch to DATA).
      For switch to XL bit-rate, the gating by TX trigger can be made inside
      the BTL since BTL has this information!
- Flag holding that "synchronization has already occured"
    - Should be set when synchronization edge occurs
    - Cleared when TSEG1 ends - in sample point!

SYNCHRONIZATION:

Hard synchronization:
- If occurs at TSEG1, then TSEG1 stays, no TX trigger is generated.
- If occurs at TSEG2, TSEG2 ends, TX trigger is generated.
- TODO: What if Hard synchronization occurs right at the sample point ?
    - The RX trigger is already active since it is registered
    - The TSEG1 should stay, but TX trigger should be generated!
    - TX / RX trigger are generated at the same time -> Is it a problem ?
        - AFAICT no -> Cross check this from standard if there are
          scenarios where this can cause problems!

Resynchronization:
- If occurs at TSEG1, it can be:
    - Ignored if SJW = 0
    - Prolong TSEG1 -> No immediate end, no immediate RX trigger, only
      affects "D" of the trigger since it will further shift it!
- If occurs at TSEG2, it can be:
    - immediately end the segment
    - shorten the preloaded segment counter



