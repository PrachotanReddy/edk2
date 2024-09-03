/** @file

  Copyright (c) 2011-2017, ARM Limited. All rights reserved.

  SPDX-License-Identifier: BSD-2-Clause-Patent

**/

#include "PeilessSec.h"

#define IS_XIP()  (((UINT64)FixedPcdGet64 (PcdFdBaseAddress) > (PcdGet64 (PcdSystemMemoryBase) + PcdGet64 (PcdSystemMemorySize) - 1)) ||\
                  ((FixedPcdGet64 (PcdFdBaseAddress) + FixedPcdGet32 (PcdFdSize)) <= PcdGet64 (PcdSystemMemoryBase)))

volatile UINT64 tlBaseAddr = 0;
volatile UINT64 tlRegX1 = 0;
volatile UINT64 tlFdtAddr = 0;
/**
  Obtain a PPI from the list of PPIs provided by the platform code.

  @param[in]  PpiGuid   GUID of the PPI to obtain
  @param[out] Ppi       Address of GUID pointer to return the PPI

  @return     Whether the PPI was obtained successfully
**/
STATIC
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
    return FALSE;
  }

  //
  // Look for a node called "memory" at the lowest level of the tree
  //
  MemoryNode = fdt_path_offset (DevTreeBase, "/memory");
  if (MemoryNode <= 0) {
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

/**
  SEC main routine.

  @param[in]  UefiMemoryBase  Start of the PI/UEFI memory region
  @param[in]  StackBase      Start of the stack
  @param[in]  StartTimeStamp  Timer value at start of execution
**/
STATIC
VOID
SecMain (
  IN  UINTN   UefiMemoryBase,
  IN  UINTN   StackBase,
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
                "FDT address at register x0:  0x%lx\n\r", tlFdtAddr
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

  // Declare the PI/UEFI memory region
  HobList = HobConstructor (
              (VOID *)UefiMemoryBase,
              FixedPcdGet32 (PcdSystemMemoryUefiRegionSize),
              (VOID *)UefiMemoryBase,
              (VOID *)StackBase // The top of the UEFI Memory is reserved for the stack
              );
  PrePeiSetHobList (HobList);

//check if the TransferList is valid and dump the contents
  if(transfer_list_check_header((VOID *)tlBaseAddr) != TL_OPS_NON) {
    transfer_list_dump((VOID *)tlBaseAddr);
  }
  else{
    DEBUG ((DEBUG_INFO | DEBUG_LOAD,"No Valid Transfer List found"));
  }
  if (fdt_check_header ((VOID *)tlFdtAddr) != 0) {
    DEBUG ((DEBUG_ERROR, "No valid DTB %p passed\n", tlFdtAddr));
    }
  else {
    NodeOffset=fdt_path_offset((VOID *)tlFdtAddr, "/");
    Compatible=fdt_getprop ((VOID *)tlFdtAddr,NodeOffset, "compatible", NULL);
    DEBUG ((DEBUG_INFO | DEBUG_LOAD,"Valid FDT with compatible property: %a\n", Compatible));

    //moving original dt to new region
    FdtSize  = fdt_totalsize ((VOID *)tlFdtAddr) + 0x6f;
    FdtPages = EFI_SIZE_TO_PAGES (FdtSize);
    NewBase  = AllocatePages (FdtPages);
    if (NewBase == NULL) {
      ASSERT (0);
    }
    fdt_open_into ((VOID *)tlFdtAddr, NewBase, EFI_PAGES_TO_SIZE (FdtPages));
      //Building FDT GuidHob
    FdtHobData = BuildGuidHob (&gFdtHobGuid, sizeof (*FdtHobData));
    if (FdtHobData == NULL) {
      ASSERT (0);
    }

    *FdtHobData = (UINTN)NewBase;
  }

  // Initialize MMU and Memory HOBs (Resource Descriptor HOBs)
  Status = MemoryPeim (UefiMemoryBase, FixedPcdGet32 (PcdSystemMemoryUefiRegionSize));
  ASSERT_EFI_ERROR (Status);

  // Create the Stacks HOB
  StacksSize = PcdGet32 (PcdCPUCorePrimaryStackSize);
  BuildStackHob (StackBase, StacksSize);

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

/**
  C entrypoint into the SEC driver.

  @param[in]  UefiMemoryBase  Start of the PI/UEFI memory region
  @param[in]  StackBase       Start of the stack
**/
VOID
CEntryPoint (
  IN  UINTN  UefiMemoryBase,
  IN  UINTN  StackBase
  )
{
  UINT64  StartTimeStamp;

  // Initialize the platform specific controllers
  ArmPlatformInitialize (ArmReadMpidr ());

  if (PerformanceMeasurementEnabled ()) {
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

  InvalidateDataCacheRange (
    (VOID *)UefiMemoryBase,
    FixedPcdGet32 (PcdSystemMemoryUefiRegionSize)
    );

  SecMain (UefiMemoryBase, StackBase, StartTimeStamp);

  // DXE Core should always load and never return
  ASSERT (FALSE);
}
