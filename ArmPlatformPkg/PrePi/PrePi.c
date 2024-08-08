/** @file

  Copyright (c) 2011-2017, ARM Limited. All rights reserved.

  SPDX-License-Identifier: BSD-2-Clause-Patent

**/

#include <PiPei.h>

#include <Library/CacheMaintenanceLib.h>
#include <Library/DebugAgentLib.h>
#include <Library/PrePiLib.h>
#include <Library/PrintLib.h>
#include <Library/PrePiHobListPointerLib.h>
#include <Library/TimerLib.h>
#include <Library/PerformanceLib.h>

#include <Ppi/GuidedSectionExtraction.h>
#include <Ppi/ArmMpCoreInfo.h>
#include <Ppi/SecPerformance.h>
#include <libfdt.h>
#include "PrePi.h"

volatile UINT64 tlBaseAddr = 0;
volatile UINT64 tlRegX1 = 0;
volatile UINT64 tlFDTAddr = 0;

#define IS_XIP()  (((UINT64)FixedPcdGet64 (PcdFdBaseAddress) > (PcdGet64 (PcdSystemMemoryBase) + PcdGet64 (PcdSystemMemorySize) - 1)) ||\
                  ((FixedPcdGet64 (PcdFdBaseAddress) + FixedPcdGet32 (PcdFdSize)) <= PcdGet64 (PcdSystemMemoryBase)))


static struct transfer_list_header *bl33_tl;
EFI_STATUS
GetPlatformPpi (
  IN  EFI_GUID  *PpiGuid,
  OUT VOID      **Ppi
  )
{
  UINTN                   PpiListSize;
  UINTN                   PpiListCount;
  EFI_PEI_PPI_DESCRIPTOR  *PpiList;
  UINTN                   Index;

  PpiListSize = 0;
  ArmPlatformGetPlatformPpiList (&PpiListSize, &PpiList);
  PpiListCount = PpiListSize / sizeof (EFI_PEI_PPI_DESCRIPTOR);
  for (Index = 0; Index < PpiListCount; Index++, PpiList++) {
    if (CompareGuid (PpiList->Guid, PpiGuid) == TRUE) {
      *Ppi = PpiList->Ppi;
      return EFI_SUCCESS;
    }
  }

  return EFI_NOT_FOUND;
}



BOOLEAN
EFIAPI
FindMemnodeInDt (
  IN VOID    *DevTreeBase,
  OUT UINT64  *SysMemBase,
  OUT UINT64  *SysMemSize
  )
{
  INT32        MemoryNode;
  INT32        AddressCells;
  INT32        SizeCells;
  INT32        Length;
  CONST UINT32  *Prop;


  if (fdt_check_header (DevTreeBase) != 0) {
    // *SysMemBase = 0x80000000;
    // *SysMemSize = 0x7C000000;
    return FALSE;
  }

  //
  // Look for a node called "memory" at the lowest level of the tree
  //
  MemoryNode = fdt_path_offset (DevTreeBase, "/memory");
  if (MemoryNode <= 0) {
    // *SysMemBase = 0x80000000;
    // *SysMemSize = 0x7C000000;
    return FALSE;
  }
  //
  // Retrieve the #address-cells and #size-cells properties
  // from the root node, or use the default if not provided.
  //
  AddressCells = 1;
  SizeCells    = 1;

  Prop = fdt_getprop (DevTreeBase, 0, "#address-cells", &Length);
  if (Length == 4) {
    AddressCells = fdt32_to_cpu (*Prop);
  }

  Prop = fdt_getprop (DevTreeBase, 0, "#size-cells", &Length);
  if (Length == 4) {
    SizeCells = fdt32_to_cpu (*Prop);
  }

  //
  // Now find the 'reg' property of the /memory node, and read the first
  // range listed.
  //
  Prop = fdt_getprop (DevTreeBase, MemoryNode, "reg", &Length);
  if (Length < (AddressCells + SizeCells) * sizeof (UINT32)) {
    // *SysMemBase = 0x80000000;
    // *SysMemSize = 0x7C000000;
    return FALSE;
  }

  if (AddressCells > 1) {
    *SysMemBase = ((UINT64)fdt32_to_cpu(Prop[0]) << 32) | fdt32_to_cpu(Prop[1]);
  }
  Prop += AddressCells;

  if (SizeCells > 1) {
    *SysMemSize = ((UINT64)fdt32_to_cpu(Prop[0]) << 32) | fdt32_to_cpu(Prop[1]);
  }
  return TRUE;
}

VOID
PrePiMain (
  IN  UINTN   UefiMemoryBase,
  IN  UINTN   StacksBase,
  IN  UINT64  StartTimeStamp
  )
{
  EFI_HOB_HANDOFF_INFO_TABLE  *HobList;
  ARM_MP_CORE_INFO_PPI        *ArmMpCoreInfoPpi;
  UINTN                       ArmCoreCount;
  ARM_CORE_INFO               *ArmCoreInfoTable;
  EFI_STATUS                  Status;
  CHAR8                       Buffer[100];
  UINTN                       CharCount;
  UINTN                       StacksSize;
  FIRMWARE_SEC_PERFORMANCE    Performance;
  VOID                        *DeviceTreeBase;
  VOID                        *NewBase;
  INT32                       NodeOffset;
  CONST CHAR8                 *Compatible;
  UINTN                       FdtSize;
  UINTN                       FdtPages;
  UINT64                      *FdtHobData;
  // If ensure the FD is either part of the System Memory or totally outside of the System Memory (XIP)
  ASSERT (
    IS_XIP () ||
    ((FixedPcdGet64 (PcdFdBaseAddress) >= PcdGet64 (PcdSystemMemoryBase)) &&
     ((UINT64)(FixedPcdGet64 (PcdFdBaseAddress) + FixedPcdGet32 (PcdFdSize)) <= (UINT64)(PcdGet64 (PcdSystemMemoryBase) + PcdGet64 (PcdSystemMemorySize) - 1)))
    );

  // Initialize the architecture specific bits
  ArchInitialize ();

  // Initialize the Serial Port
  SerialPortInitialize ();
  CharCount = AsciiSPrint (
                Buffer,
                sizeof (Buffer),
                "UEFI firmware (version %s built at %a on %a)\n\r",
                (CHAR16 *)PcdGetPtr (PcdFirmwareVersionString),
                __TIME__,
                __DATE__
                );
  SerialPortWrite ((UINT8 *)Buffer, CharCount);
  CharCount = AsciiSPrint (
                  Buffer,
                  sizeof (Buffer),
                  "Register Values from BL31 \n\r"
                  );
    SerialPortWrite ((UINT8 *)Buffer, CharCount);
    CharCount = AsciiSPrint (
                  Buffer,
                  sizeof (Buffer),
                  "FDT address at register x0:  0x%lx\n\r", tlFDTAddr
                  );
    SerialPortWrite ((UINT8 *)Buffer, CharCount);
    CharCount = AsciiSPrint (
                  Buffer,
                  sizeof (Buffer),
                  "TL signature at register x1:  0x%lx\n\r", tlRegX1
                  );
    SerialPortWrite ((UINT8 *)Buffer, CharCount);
    CharCount = AsciiSPrint (
                  Buffer,
                  sizeof (Buffer),
                  "Reserved must be zero at register x2\n\r"
                  );
    SerialPortWrite ((UINT8 *)Buffer, CharCount);
    CharCount = AsciiSPrint (
                  Buffer,
                  sizeof (Buffer),
                  "tl_base_pa at register x3:  0x%lx\n\r", tlBaseAddr
                  );
    SerialPortWrite ((UINT8 *)Buffer, CharCount);

  // Initialize the Debug Agent for Source Level Debugging
  InitializeDebugAgent (DEBUG_AGENT_INIT_POSTMEM_SEC, NULL, NULL);

  SaveAndSetDebugTimerInterrupt (TRUE);
  DEBUG ((DEBUG_INFO | DEBUG_LOAD,"System Memory Base found in fdt from TF-a: 0x%lx\n", PcdGet64(PcdSystemMemoryBase)));
  DEBUG ((DEBUG_INFO | DEBUG_LOAD,"System Memory Size found in fdt from TF-a: 0x%lx\n", PcdGet64(PcdSystemMemorySize)));
  
  // Declare the PI/UEFI memory region
  HobList = HobConstructor (
              (VOID *)UefiMemoryBase,
              FixedPcdGet32 (PcdSystemMemoryUefiRegionSize),
              (VOID *)UefiMemoryBase,
              (VOID *)StacksBase // The top of the UEFI Memory is reserved for the stacks
              );

  PrePeiSetHobList (HobList);

  if(transfer_list_check_header((void *)tlBaseAddr) != TL_OPS_NON) {
    bl33_tl = (VOID *)tlBaseAddr; /* saved TL address from TF-A */
    transfer_list_dump((VOID *)tlBaseAddr);
    DeviceTreeBase = (VOID *)tlFDTAddr;
    if (fdt_check_header (DeviceTreeBase) != 0) {
    DEBUG ((DEBUG_ERROR, "Invalid DTB %p passed\n", DeviceTreeBase));    }
    else {
      NodeOffset=fdt_path_offset(DeviceTreeBase, "/");
      Compatible=fdt_getprop (DeviceTreeBase,NodeOffset, "compatible", NULL);
      DEBUG ((DEBUG_INFO | DEBUG_LOAD,"Compatible property on FDT: %a\n", Compatible));

      //adding fdt hob
      FdtSize  = fdt_totalsize (DeviceTreeBase) + 0x6f;
      FdtPages = EFI_SIZE_TO_PAGES (FdtSize);
      NewBase  = AllocatePages (FdtPages);
      if (NewBase == NULL) {
        ASSERT (0);
      }
      fdt_open_into (DeviceTreeBase, NewBase, EFI_PAGES_TO_SIZE (FdtPages));
    }
  }
  else{
    DEBUG ((DEBUG_INFO | DEBUG_LOAD,"No Valid Transfer List found"));
  }
  // Initialize MMU and Memory HOBs (Resource Descriptor HOBs)
  Status = MemoryPeim (UefiMemoryBase, FixedPcdGet32 (PcdSystemMemoryUefiRegionSize));
  ASSERT_EFI_ERROR (Status);

  // Create the Stacks HOB (reserve the memory for all stacks)
  if (ArmIsMpCore ()) {
    StacksSize = PcdGet32 (PcdCPUCorePrimaryStackSize) +
                 ((FixedPcdGet32 (PcdCoreCount) - 1) * FixedPcdGet32 (PcdCPUCoreSecondaryStackSize));
  } else {
    StacksSize = PcdGet32 (PcdCPUCorePrimaryStackSize);
  }

  BuildStackHob (StacksBase, StacksSize);

  // TODO: Call CpuPei as a library
  BuildCpuHob (ArmGetPhysicalAddressBits (), PcdGet8 (PcdPrePiCpuIoSize));

  if (ArmIsMpCore ()) {
    // Only MP Core platform need to produce gArmMpCoreInfoPpiGuid
    Status = GetPlatformPpi (&gArmMpCoreInfoPpiGuid, (VOID **)&ArmMpCoreInfoPpi);

    // On MP Core Platform we must implement the ARM MP Core Info PPI (gArmMpCoreInfoPpiGuid)
    ASSERT_EFI_ERROR (Status);

    // Build the MP Core Info Table
    ArmCoreCount = 0;
    Status       = ArmMpCoreInfoPpi->GetMpCoreInfo (&ArmCoreCount, &ArmCoreInfoTable);
    if (!EFI_ERROR (Status) && (ArmCoreCount > 0)) {
      // Build MPCore Info HOB
      BuildGuidDataHob (&gArmMpCoreInfoGuid, ArmCoreInfoTable, sizeof (ARM_CORE_INFO) * ArmCoreCount);
    }
  }

  // Store timer value logged at the beginning of firmware image execution
  Performance.ResetEnd = GetTimeInNanoSecond (StartTimeStamp);

  // Build SEC Performance Data Hob
  BuildGuidDataHob (&gEfiFirmwarePerformanceGuid, &Performance, sizeof (Performance));

  // Set the Boot Mode
  SetBootMode (ArmPlatformGetBootMode ());

  // Initialize Platform HOBs (CpuHob and FvHob)
  Status = PlatformPeim ();
  ASSERT_EFI_ERROR (Status);
  
  //Building FDT GuidHob
  FdtHobData = BuildGuidHob (&gFdtHobGuid, sizeof (*FdtHobData));
  if (FdtHobData == NULL) {
    ASSERT (0);
  }

  *FdtHobData = (UINTN)NewBase;

  // Now, the HOB List has been initialized, we can register performance information
  PERF_START (NULL, "PEI", NULL, StartTimeStamp);

  // SEC phase needs to run library constructors by hand.
  ProcessLibraryConstructorList ();

  // Assume the FV that contains the SEC (our code) also contains a compressed FV.
  Status = DecompressFirstFv ();
  ASSERT_EFI_ERROR (Status);

  // Load the DXE Core and transfer control to it
  Status = LoadDxeCoreFromFv (NULL, 0);
  ASSERT_EFI_ERROR (Status);
}

VOID
CEntryPoint (
  IN  UINTN  MpId,
  IN  UINTN  UefiMemoryBase,
  IN  UINTN  StacksBase
  )
{
  UINT64  StartTimeStamp;

  // Initialize the platform specific controllers
  ArmPlatformInitialize (MpId);

  if (ArmPlatformIsPrimaryCore (MpId) && PerformanceMeasurementEnabled ()) {
    // Initialize the Timer Library to setup the Timer HW controller
    TimerConstructor ();
    // We cannot call yet the PerformanceLib because the HOB List has not been initialized
    StartTimeStamp = GetPerformanceCounter ();
  } else {
    StartTimeStamp = 0;
  }

  // Data Cache enabled on Primary core when MMU is enabled.
  ArmDisableDataCache ();
  // Invalidate instruction cache
  ArmInvalidateInstructionCache ();
  // Enable Instruction Caches on all cores.
  ArmEnableInstructionCache ();

  // Define the Global Variable region when we are not running in XIP
  if (!IS_XIP ()) {
    if (ArmPlatformIsPrimaryCore (MpId)) {
      if (ArmIsMpCore ()) {
        // Signal the Global Variable Region is defined (event: ARM_CPU_EVENT_DEFAULT)
        ArmCallSEV ();
      }
    } else {
      // Wait the Primary core has defined the address of the Global Variable region (event: ARM_CPU_EVENT_DEFAULT)
      ArmCallWFE ();
    }
  }

  // If not primary Jump to Secondary Main
  if (ArmPlatformIsPrimaryCore (MpId)) {
    InvalidateDataCacheRange (
      (VOID *)UefiMemoryBase,
      FixedPcdGet32 (PcdSystemMemoryUefiRegionSize)
      );

    // Goto primary Main.
    PrimaryMain (UefiMemoryBase, StacksBase, StartTimeStamp);
  } else {
    SecondaryMain (MpId);
  }

  // DXE Core should always load and never return
  ASSERT (FALSE);
}
