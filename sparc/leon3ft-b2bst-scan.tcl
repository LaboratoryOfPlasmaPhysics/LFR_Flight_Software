#! /usr/bin/tclsh
#
# TCL script to scan disassembled programs against the "LEON3FT Stale Cache
# Entry After Store with Data Tag Parity Error" errata, GRLIB-TN-0009.
#
# The following Cobham components are affected by GRLIB-TN-0009:
# - GR712RC
# - LEON3FT-RTAX (all versions)
# - UT699
# - UT699E
# - UT700
#
# For information on how custom LEON3 systems may be affected, see
# GRLIB-TN-0009.
#
# Copyright (C) 2016, Cobham Gaisler
#
# Usage: sparc-elf-objdump -d program | leon3ft-b2bst-scan.tcl
#
# Command line parameters:
#   -nonote: Do not print NOTE messages.
#   -noinfo: Do not print INFO messages.
#   -silent: Do not print NOTE or INFO messages.
#
# Return values:
#   0: No potential error locations found.
#   1: At least one potential error location found.
#
# Rev. history:
#   rev 1, MH, Initial revision, based on ut699scan.tcl. Add scanning for
#              store-store errata, reorganize comments
#   rev 2, MA, Restruction and clean-up, fixed Sequence B.
#   rev 3, MA, Added decoding of casa instruction, exit status and verbosity
#              parameters.
#   rev 4, MA, Reduced false warnings due to Store into Alternate space
#              instructions which can not trig the errata. Script renamed to
#              leon3ft-b2bst-scan.tcl.
#
# Note 1: Two modes are available with respect to call/ret (dslot_mode):
#   1) ret, and call are assumed not to have stores in their delay slots.
#      The tool will warn if any such operations are found in the delay
#      slot of ret,call during the scan
#   0) ret and call may have stores in their delay slots
#      The tool will not warn if any such operations are found in the
#      delay slot during the scan. Instead it will assume this
#      possibility exists when evaluating insns following a function call
#

### CONFIGURATION
set b2bst_seqa_scan 1; # enable/disable scanning for Sequence A
set b2bst_seqb_scan 1; # enable/disable scanning for Sequence B
# set dslot_mode    0; # assume store can be in delay slot of call,jmpl
set dslot_mode      1; # do not allow store in delay slot of call,jmpl
###

# Mask values for types of output messages
set MSG_INFO 1; # Information on things to check manually.
set MSG_NOTE 2; # Notices on branch following
set msglevel [expr {$MSG_INFO | $MSG_NOTE}]

set errata_desc "LEON3FT Stale Cache Entry After Store with Data Tag Parity Error"
set util_rev 4
set util_date "20170215"

set b2bst_seqa_desc "Sequence A"
set b2bst_seqb_desc "Sequence B"

set op_count 0
set lineskip_count 0
set line_count 0
set err_count 0
set fn_count 0
set fninst_count 0

proc puts_info {str} {
    global msglevel MSG_INFO
    if {$msglevel & $MSG_INFO} { puts "INFO: $str" }
}

proc puts_note {str} {
    global msglevel MSG_NOTE
    if {$msglevel & $MSG_NOTE} { puts "NOTE: $str" }
}

# ----------------------------------------------------------------------------
# Routine that decodes specified opcode and returns list of
# 0 :optype       Class of opcode
#        one of call,jmpl,branch,sethi,ld,st,atomic,alu,fpop,save,restore
# 1 :target-reg   Register modified by op
# 2 :source-reg1  First source register (rs1)
# 3 :source-reg2  Second source register (rs2)
# 4 :store-reg    Source register for store data
#        regs are numbered 0-31 integer regs, 32-63 FP regs
# 5 :double-flag
#        ORed mask: 8) target-reg is DP, 4) sreg1 is DP, 2) sreg2 is DP, 1) store-reg is DP
# 6 :immed        Immediate operand
# 7 :nnpc1        Next instructions nPC if branch not taken (normal case), normally nPC+4
# 8 :nnpc2        Next instruction's nPC if branch is taken, if not branch then nnpc2=nnpc1
# 9 :annul
#        ORed mask: next inst annulled if 2) branch not taken 1) branch taken
# 10:bubbles      Number of extra cycles spent in decode stage (bubbles inserted)
proc decode_inst { opcode {pc "0x00000000"} {npc "0x00000004"}} {
    # Some "sta" instructions do not trig the errata. The related ASI:s are
    # defined here.
    set safeasi [list 0x02 0x03 0x04 0x0c 0x0d 0x0e 0x0f 0x10 0x11 0x13 0x14 0x15 0x18 0x19 0x1c 0x1d 0x1e]
    #Default values if no cond matches
    set opname "unknown"
    set destreg 0
    set sreg1 0
    set sreg2 0
    set streg 0
    set dblflag 0
    set immed 0

    if { [string is integer $npc] } {
        set nnpc1 [format "0x%08x" [ expr {$npc+4} ] ]
    } else {
        set nnpc1 "$npc+4"
    }
    set nnpc2 $nnpc1
    set annul 0
    set bubbles 0
    #Decode
    set op1 [ expr {$opcode >> 30} ]
    if { $op1 == 1 } {
        set opname "call"
        set disp [ expr { ($opcode & 0x3fffffff)<<2 } ]
        set nnpc1 [ format "0x%08x" [ expr { $pc + $disp } ] ]
        set nnpc2 $nnpc1
    } elseif { $op1 == 0 } {
        set op2 [ expr {($opcode >> 22) & 7} ]
        # NOTE: 2=Bicc, 6=FBcc, 7=CBcc
        if { $op2 == 2 || $op2 == 6 } {
            set opname "branch"
            set disp [expr { ($opcode & 0x3fffff) << 2 } ]
            if { $disp > 0x800000 } { set disp [expr {$disp - 0x1000000}] }
            # Todo check cond,annul, handle always or never
            set nnpc2 [ format "0x%08x" [ expr { $pc + $disp } ] ]
            set cond [ expr { ($opcode >> 25) & 15 } ]
            set annulbit [ expr { $opcode & 0x20000000 } ]
            if { $cond == 0 } { # Branch never
                set nnpc2 $nnpc1
                if { $annulbit } { set annul 3 }
            } elseif { $cond == 8 } { # Branch always
                set nnpc1 $nnpc2
                if { $annulbit } { set annul 3 }
            } else { # True cond branch
                if { $annulbit } { set annul 2 }
            }
        } elseif { $op2 == 4 } {
            set opname "sethi"
            set destreg [ expr { ($opcode >> 25) & 31 } ]
            set immed [ expr { ($opcode & 0x3fffff) << 10 } ]
        }
    } elseif { $op1 == 3 } {
        # SPARC V8 Table F-4
        # set opname "mem" - changed to use ld/st/atomic instead
        set op3 [ expr { ($opcode >> 19) & 63 } ]
        set rs1 [ expr { (($opcode >> 14) & 31) } ]
        set rs2 [ expr { (($opcode      ) & 31) } ]
        set rd  [ expr { (($opcode >> 25) & 31) } ]
        set sreg1 $rs1
        if { $opcode & 0x2000 } {
            set immed [ expr { $opcode & 0x1fff } ]
            if { $immed > 0xfff } { set immed [ expr { $immed-(0x2000) } ] }
        } else {
            set sreg2 $rs2
        }
        if { (($op3 & 13) == 13) || ($op3 == 0x3c) } {
            # Atomic ldst/swap/casa
            set opname "atomic"
            set destreg $rd
            set streg $rd
        } else {
            # Adjust reg no if FP ld/st (op3[5]=1)
            if { ($op3 & 0x20) != 0 } { set rd [expr {32 + $rd} ] }
            # Check if read or write op3[2]
            if { ($op3 & 4) != 0 } {
                set opname "st"
                set streg $rd
                set allow_safe_sta true
                if { $allow_safe_sta && (($op3 & 0x3c) == 0x14) } {
                    # op3 is 0x14, 0x15, 0x16 or 0x17 (sta, stba, stha or stda)
                    # (if i = 0)
                    set asi [expr { (($opcode >> 5) & 0xff) }]
                    set safematch [lsearch -exact -integer $safeasi $asi]
                    if {0 <= $safematch} {
                        set opname "sta_safe"
                    }
                }
            } else {
                set opname "ld"
                set destreg $rd
            }
            # Check size op3[1:0]
            if { ($op3 & 3) == 3 } {
                if { $opname eq "st" } { set dblflag 1 } else { set dblflag 8 }
            }
        }
    } else {
        set op3 [ expr { ($opcode >> 19) & 63 } ]
        if { $op3 == 0x34 || $op3 == 0x35 } {
            set opname "fpop"
            set opf [ expr { ($opcode >> 5) & 0x1ff } ]
            set rs1 [ expr { 32 + (($opcode >> 14) & 31) } ]
            set rs2 [ expr { 32 + (($opcode      ) & 31) } ]
            set rd  [ expr { 32 + (($opcode >> 25) & 31) } ]
            # opf[1:0] corresponds to operand size of rs2
            # opf[7:6]=11 is conv op, opf[3:2] gives opsize of rd, rs1 unused
            # opf[7:6]=00 is single-operand op, rd same size as rs2, rs1 unused
            # opf[7:4]=0100 regular op, rs1,rd same size as rs2
            # opf[7:4]=0101 comp op, rd unused, rs1 same size as rs2
            # opf[7:4]=0110 conv+mul, opf[3:2] gives opsize of rd, rs1 same size as rs2
            set sreg2 $rs2
            if { ($opf & 3) > 1 } { set dblflag 2 }
            if { ($opf & 0xc0) == 0x40 } {
                set sreg1 $rs1
                if { $dblflag & 2 } { set dblflag [ expr { $dblflag | 4 } ] }
            }
            if { ($opf & 0xc0) == 0xc0 || ($opf & 0xf0) == 0x60 } {
                set destreg $rd
                if { ($opf & 0xc) > 4 } { set dblflag [ expr { $dblflag | 8 } ] }
            } elseif { ($opf & 0xf0) != 0x50 } {
                set destreg $rd
                if { $dblflag & 2 } { set dblflag [ expr { $dblflag | 8 } ] }
            }
        } else {
            set opname "alu"
            set destreg  [ expr { (($opcode >> 25) & 31) } ]
            set sreg1 [ expr { (($opcode >> 14) & 31) } ]
            if { $opcode & 0x2000 } {
                set immed [ expr { $opcode & 0x1fff } ]
                if { $immed > 0xfff } { set immed [ expr { $immed-(0x2000) } ] }
            } else {
                set sreg2 [ expr { (($opcode      ) & 31) } ]
            }
            if { $op3 == 0x38 } {
                set opname "jmpl"
                set nnpc1 "jmpaddr"
                set nnpc2 "jmpaddr"
                set bubbles 2
            } elseif { $op3 == 0x3c } {
                set opname "save"
            } elseif { $op3 == 0x3d } {
                set opname "restore"
            }
        }
    }
    #Return values
    return [list $opname $destreg $sreg1 $sreg2 $streg $dblflag $immed $nnpc1 $nnpc2 $annul $bubbles]
}

proc is_store { opcode } {
    set op1 [ expr { ($opcode >> 30) } ]
    set op3 [ expr { ($opcode >> 19) & 63 } ]
    return [expr {($op1 == 3) && (($op3 & 0xc) == 0x4)}]
}

# -------------------------------------------------------------------------------------

proc unpack { l args } {
    for { set i 0 } { $i < [llength $args] } { incr i } {
        uplevel 1 "set [lindex $args $i]  [lindex $l $i]"
    }
}

# --------- Postprocessing on whole function

# Number of instructions printed is starti + 2
proc print_trace { fnpc state fnarr seqstart {starti 4}} {
    upvar $fnarr fnops

    puts "  #  PC          Opcode"
    set cnt [expr {$seqstart-$starti}]
    for { set x $starti } { $x > -2 } { set x [expr {$x-1}] } {
        incr cnt
        if { $cnt > 0 } { set cntv $cnt } else { set cntv " " }
        if { $x >= 0 } { set pc [lindex $state $x] } else { set pc $fnpc }
        if { $pc eq "?" || $pc eq "call" || $pc eq "ret" || $pc eq "dslot" } {
            puts "  $cntv  (other fn)  $pc"
        } else {
            if { [string index $pc 0] eq "A" } {
                set pc [string range $pc 1 end]
                set annul " (annulled)"
            } else {
                set annul ""
            }
            if { [info exists fnops($pc)] } {
                set op $fnops($pc)
            } else {
                set op [list "0x???????? (outside func)"]
            }
            puts "  $cntv  $pc  [join $op] $annul"
        }
    }
    puts ""
}

# Routine called by scan_wholefunc_main to check
# instruction sequences described in GRLIB-TN-009, Issue 1.0
#
# "Sequence A"
# 1. store of word size or less (st / stb / sth / stf)
# 2. any single instruction that is not a load or store
# 3. any store instruction (st / stb / sth / stf / std / stdf)
#
# "Sequence B"
# 1. store of double word size (std / stdf)
# 2. any store instruction (st / stb / sth / stf / std / stdf)
proc scan_b2bst { fnname fnpc fnop opdec fnarr state } {
    global b2bst_seqa_scan
    global b2bst_seqb_scan
    upvar $fnarr fnops
    global dslot_mode

    # Check latest 3 insn if they can be st/std/regular
    set pca [list $fnpc [lindex $state 0] [lindex $state 1]]
    set can_be_arr [list]
    for { set i 0 } { $i < 3 } { incr i } {
        set can_be [list 0 0 0] ; # st std regular
        set pcv [lindex $pca $i]
        if { $pcv eq "?" } {
            set can_be [list 1 1 1]
        } elseif { $pcv eq "dslot" } {
            set can_be [list [expr {1-$dslot_mode}] [expr {1-$dslot_mode}] 1]
        } elseif { [string index $pcv 0] eq "A" || $pcv eq "call" || $pcv eq "ret" } {
            set can_be [list 0 0 1]
        } else {
            set op $fnops($pcv)
            set ophex [lindex $op 0]
            set opdec [decode_inst $ophex]
            set optype [lindex $opdec 0]
            set opstorereg [lindex $opdec 4]
            set opdblflag [lindex $opdec 5]
            if { $optype eq "st" && ($opdblflag & 1) } {
                set can_be [list 0 1 0]
            } elseif { $optype eq "st" } {
                set can_be [list 1 0 0]
            } elseif { $optype ne "ld" && $optype ne "atomic" } {
                # Any single instruction that is not a load or store
                set can_be [list 0 0 1]
            }
        }
        lappend can_be_arr $can_be
    }
    # can_be_arr has at index
    # - at index 2: previous previous instruction
    # - at index 1: previous instruction
    # - at index 0: current instruction
    set pp_insn [lindex $can_be_arr 2]
    set p_insn [lindex $can_be_arr 1]
    set c_insn [lindex $can_be_arr 0]
    # Sequence A
    # 1. store of word size or less
    # 2. any single instruction that is not a load or store
    # 3. any store instruction
    if {
    (
        $b2bst_seqa_scan &&
        [lindex $pp_insn 0] &&
        [lindex $p_insn 2] &&
        ([lindex $c_insn 0] || [lindex $c_insn 1])
    ) } {
        puts "WARNING: Possible sequence matching LEON3FT b2bst errata (sequence A) in function $fnname"
        print_trace $fnpc $state fnops 1
        puts ""
        return 1
    }

    # Sequence B
    # 1. any store instruction
    # 2. store of double word size
    if {
    (
        $b2bst_seqb_scan &&
        [lindex $p_insn 1] &&
        ([lindex $c_insn 0] || [lindex $c_insn 1])
    ) } {
        puts "WARNING: Possible sequence matching LEON3FT b2bst errata (sequence B) in function $fnname"
        print_trace $fnpc $state fnops 0
        puts ""
        return 1
    }
    return 0
}


# Main recursive function doing scan
# fnname - funcion name
# fnpc, fnnpc, PC/nPC of current insn (8-digit hex, 0x prefix) prefix also with A if annulled,
# fnarr - array/hash-table, see descr for scan_wholefunc
# dpdist - number of SP FPOPs done minimum since last DP FPOP
# state - List of 5 last instruction addresses (hex w 0x prefix), special values "?"=unknown "call"=call from calling proc
# smap - Hash table used to track where we have been already
proc scan_wholefunc_main { fnname fnpc fnnpc fnarr savelevel dpdist state smap lastoptype } {
    global dslot_mode
    upvar $fnarr fnops
    upvar $smap statemap

    set r 0

    # puts "entering scan_wholefunc_main: PC:$fnpc nPC:$fnnpc dpdist:$dpdist savelevel:$savelevel state:$state"
    # puts "fnops exists [info exists fnops] [array exists fnops] [array names fnops]"
    while {1} {
        # puts "scan_wholefunc_main: PC:$fnpc nPC:$fnnpc dpdist:$dpdist savelevel:$savelevel state:$state"
        set statedesc [list $fnpc $fnnpc $savelevel $dpdist $state]
        if { [info exists statemap($statedesc)] } {
            # Already been here
            return $r
        }
        set statemap($statedesc) 1

        set fnpca $fnpc
        set pcannul 0
        if { [string index $fnpc 0] eq "A" } {
            set pcannul 1
            set fnpc [string range $fnpc 1 end]
            set ophex "0x01000000"
        } else {
            if {![info exists fnops($fnpc)]} {
                puts_note "$fnname: Execution leaves function without return";
                return $r
            }

            set op $fnops($fnpc)
            set ophex [lindex $op 0]
            set opname [lindex $op 1]
        }

        set opdec [ decode_inst $ophex $fnpc $fnnpc ]
        unpack $opdec optype treg sreg1 sreg2 streg dblflag immed nnpc1 nnpc2 annul bubbles

        set prev_savelevel $savelevel
        if { $optype eq "fpop" } {
            # update dpdist
            if { ($dblflag & 6) == 0 } {
                if { $dpdist < 2 } { incr dpdist }
            } else {
                set dpdist 0
            }
        } elseif { $optype eq "save" } {
            if { $savelevel > 9 } {
                puts_note "$fnname: More than 10 deep saves (possibly regfile clear loop)"
            } else {
                incr savelevel
            }
            # puts "save at $fnpc, new savelevel: $savelevel"
        } elseif { $optype eq "restore" } {
            if { $savelevel < -9 } {
                puts_note "$fnname: More than 10 deep restores (possibly regfile clear loop)"
            } else {
                if { $savelevel == 0 } {
                    puts_note "$fnname: More restore than save instructions"
                }
                set savelevel [ expr { $savelevel-1} ]
            }
            # puts "restore at $fnpc, new savelevel: $savelevel"
        }
        if { true && ($optype eq "st") } {
            if { [scan_b2bst $fnname $fnpc $op $opdec fnops $state] } { incr r }
        }

        # Handle jmpl and call
        # If in delay slot of jmpl:
        #   If ret or retl, stop recursing further
        #   If other jmpl, check savelevel if it will return or not
        # If in delay slot of call, set dpdist to 0 (called proc may have done DP operation)
        set is_call 0
        if {
        (
            $dslot_mode != 0 &&
            ($lastoptype eq "branch" || $lastoptype eq "jmpl" || $lastoptype eq "call")
        ) } {
            if { $optype eq "st" } {
                puts "WARNING: $fnname: $opname in delay slot of branch/call/ret at $fnpc may cause errata"
                print_trace $fnpc $state fnops -1 1
                puts ""
                incr r
            }
        }
        if { $lastoptype eq "jmpl" } {
            set linst [lindex $state 0]
            set q $fnops($linst)
            set qh [lindex $q 0]
            set qdec [ decode_inst $qh $linst $fnpc ]
            unpack $qdec qdtype qdtreg qdsreg1 qdsreg2 qdstreg qddblflag qdimmed qdnnpc1 qdnnpc2 qdannul
            if { $qdtreg==15 && $prev_savelevel==1 && $savelevel==1 } {
                # Function call via pointer
                incr is_call
            } elseif { $qdtreg==0 && $savelevel==0 } {
                # Return or tail recursion
                return $r
            } else {
                # Computed goto or some other non-standard construct
                puts_info "$fnname: Unable to trace jmpl at $linst - check manually"
                return $r
            }
        } elseif { $lastoptype eq "call" } {
            if { $savelevel==0 && $prev_savelevel==1 } {
                # call,restore tail recursion into other fn
                return $r
            } elseif { $savelevel==0 && $prev_savelevel==0 && $treg == 15 } {
                puts_note "$fnname: Leaf tail recursion construct at $fnpc"
                return $r
            } else {
                incr is_call
            }
        }
        # Update history
        for { set z 0 } { $z <= $bubbles } { incr z } {
            set state [concat $fnpca [lrange $state 0 4] ]
        }
        if { $is_call } {
            if { $optype eq "branch" || $optype eq "call" || $optype eq "jmpl" } {
                puts_info "$fnname: jump in delay slot of call - check manually"
            }
            # Setup state after call has returned
            set dpdist 0
            set state [list "dslot" "ret" "ret" "ret" "?" ]
            set optype "?"; # for lastoptype
            set fnnpc [ format "0x%08x" [expr {$fnpc+4}]]
            set nnpc1 [ format "0x%08x" [expr {$fnpc+8}]]
            set nnpc2 $nnpc1
            set annul 0
        }
        # Handle annullment
        if { ($annul & 2) != 0 } { set npc1 "A$fnnpc" } else { set npc1 $fnnpc }
        if { ($annul & 1) != 0 } { set npc2 "A$fnnpc" } else { set npc2 $fnnpc }
        # Recurse if multiple nPC:s, tail recurse for last element
        if { $nnpc1 != $nnpc2 || $npc1 != $npc2 } {
            # puts "Recursion nnpc:$nnpc2!"
            set q [scan_wholefunc_main $fnname $npc1 $nnpc2 fnops $savelevel $dpdist $state statemap $optype]
            set r [expr {$r+$q}]
        }
        set fnpc $npc1
        set fnnpc $nnpc1
        set lastoptype $optype
    }
}

# Whole function scan for "LEON3FT Stale Cache After Store with Data Tag Parity Error" errata
# fnname - function name, fnaddr - function addres
# fnarr - name of array/hash-table containing operations
#   key to the array is the address as 8-digit hex number prefixed with 0x
#   value is a list containing 0:opcode, 1:mnemonic, 2:args
proc scan_wholefunc { fnname fnaddr fnarr } {
    global fn_count fninst_count;
    upvar $fnarr fnops

    if { $fnname eq ".text" } {
        puts "WARNING: The disassembled binary appears to be stripped, the script can not scan stripped binaries"
    }

    # Does the fn not contain any store at all, then we can skip it
    set sid [array startsearch fnops]
    set found_st 0
    while { !$found_st } {
        set op [array nextelement fnops $sid]
        if { $op eq "" } { array donesearch fnops $sid; break }
        set ophex [lindex $fnops($op) 0]
        # puts "$ophex"
        if { [is_store $ophex] } { incr found_st }
    }
    if { !$found_st } {
        # puts "No store in $fnname!"
        return 0
    }

    # -- Function contains store, proceed with scanning
    # puts "Checking $fnname"
    set pc "0x$fnaddr"
    # puts "pc: $pc"
    set npc [format "0x%08x" [expr {$pc+4}]]
    array unset statemap
    array set statemap ""
    set ec [scan_wholefunc_main $fnname $pc $npc fnops 0 0 [list "dslot" "call" "?" "?" "?" "?"] statemap "?"]
    # Update fn_count/fninst_count
    incr fn_count
    array unset itagchk
    array set itagchk ""
    set sid [array startsearch statemap]
    while { [array anymore statemap $sid] } {
        set x [array nextelement statemap $sid]
        set iaddr [lindex $x 0]
        # puts "iaddr: $iaddr"
        if { [info exists fnops($iaddr)] && ![info exists itagchk($iaddr)]} {
            incr fninst_count
            set itagchk($iaddr) 1
        }
    }
    array donesearch statemap $sid
    return $ec
}


# --------- Main routine

if { ! [info exists sourcing] } {

# Parse command line
for {set i 0} {$i < $argc} {incr i} {
        set arg [lindex $argv $i]
        puts "arg=$arg"
        if {"-noinfo" eq $arg} {
                set msglevel [expr {$msglevel & ~($MSG_INFO)}]
        }
        if {"-nonote" eq $arg} {
                set msglevel [expr {$msglevel & ~($MSG_NOTE)}]
        }
        if {"-silent" eq $arg} {
                set msglevel 0
        }
}

puts "\n$errata_desc errata scanning utility, rev $util_rev ($util_date)"

puts "Searching objdump -d output on standard input for:"
if {$b2bst_seqa_scan} {
    puts " - $b2bst_seqa_desc"
}
if {$b2bst_seqb_scan} {
    puts " - $b2bst_seqb_desc"
}
puts ""

set fn_valid 0
while {! [eof stdin]} {
    set l [gets stdin]
    incr line_count
    # puts "line $l"
    # Remove comment after '!'
    set x [string first "!" "$l"]
    if { $x >= 0 } then { set l [string range "$l" 0 $x] }
    # check if entry point / function start
    set m [regexp {^ *([0-9A-Fa-f]+) <([^ ]+)>:} "$l" t1 addr symname]
    if { $m > 0 } then {
        # puts "Function name=$symname addr=$addr"
        if { $fn_valid } {
            # puts $curfn_ops(0x40000000)
            set q [scan_wholefunc $curfn $curfn_addr curfn_ops]
            # puts "q=$q err_count=$err_count"
            set err_count [ expr { $err_count + $q } ]
        }
        set curfn "$symname"
        set curfn_addr "$addr"
        array unset curfn_ops
        array set curfn_ops ""
        set fn_valid 0
    } else {
        # Check if opcode
        set m [regexp {^ *([0-9A-Fa-f]+): *\t(.. .. .. ..) *\t([^ ]+)(.*)$} "$l" t1 addr hexcode opname opargs]
        if { $m > 0 } then {
            # puts "Opcode name=$opname args=$opargs addr=$addr"
            incr op_count
            set addr [format "%08x" 0x$addr]
            set hexcode [string map {" " ""} $hexcode]
            # puts "setting curfn_ops 0x$addr"
            set curfn_ops(0x$addr) [list "0x$hexcode" $opname $opargs "" ""]
            incr fn_valid
        } else {
            incr lineskip_count
        }
    }
}
if { $fn_valid } {
    set q [scan_wholefunc $curfn $curfn_addr curfn_ops]
    set err_count [ expr { $err_count + $q } ]
}

puts "\nObjdump lines processed: $line_count, lines skipped: $lineskip_count"
puts "Functions scanned: ${fn_count}, reachable instruction count: ${fninst_count}"
puts "Potential error locations found: $err_count\n"

if {$err_count == 0} {
        exit 0
} else {
        exit 1
}

}

