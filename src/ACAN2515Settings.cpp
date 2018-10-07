//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// A CAN driver for MCP2515
// by Pierre Molinaro
// https://github.com/pierremolinaro/acan2515
//
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

#include <ACAN2515Settings.h>

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//    CAN Settings
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

ACANSettings2515::ACANSettings2515 (const uint32_t inQuartzFrequency,
                                    const uint32_t inWhishedBitRate,
                                    const uint32_t inTolerancePPM) :
mQuartzFrequency (inQuartzFrequency) {
  if (mDesiredBitRate != inWhishedBitRate) {
    mDesiredBitRate = inWhishedBitRate ;
    uint32_t TQCount = 25 ; // TQCount: 5 ... 25
    uint32_t smallestError = UINT32_MAX ;
    uint32_t bestBRP = 64 ; // Setting for slowest bit rate
    uint32_t bestTQCount = 25 ; // Setting for slowest bit rate
    uint32_t BRP = mQuartzFrequency / inWhishedBitRate / TQCount / 2 ;
  //--- Loop for finding best BRP and best TQCount
    while ((TQCount >= 5) && (BRP <= 64)) {
    //--- Compute error using BRP (caution: BRP should be > 0)
      if (BRP > 0) {
        const uint32_t error = (mQuartzFrequency / TQCount / BRP / 2) - inWhishedBitRate ; // error is always >= 0
        if (error < smallestError) {
          smallestError = error ;
          bestBRP = BRP ;
          bestTQCount = TQCount ;
        }
      }
    //--- Compute error using BRP+1 (caution: BRP+1 should be <= 64)
      if (BRP < 64) {
        const uint32_t error = inWhishedBitRate - (mQuartzFrequency / TQCount / 2 / (BRP + 1)) ; // error is always >= 0
        if (error < smallestError) {
          smallestError = error ;
          bestBRP = BRP + 1 ;
          bestTQCount = TQCount ;
        }
      }
    //--- Continue with next value of TQCount
      TQCount -- ;
      BRP = mQuartzFrequency / inWhishedBitRate / TQCount / 2 ;
    }
  //--- Set the BRP
    mBitRatePrescaler = (uint8_t) bestBRP ;
  //--- Compute PS2
    const uint32_t PS2 = (bestTQCount + 1) / 3 ; // Always 2 <= PS2 <= 8
    mPhaseSegment2 = (uint8_t) PS2 ;
  //--- Compute the remaining number of TQ once PS2 and SyncSeg are removed
    const uint32_t propSegmentPlusPhaseSegment1 = bestTQCount - PS2 - 1 /* Sync Seg */ ;
  //--- Set PS1 to half of remaining TQCount
    const uint32_t PS1 = propSegmentPlusPhaseSegment1 / 2 ; // Always 1 <= PS1 <= 8
    mPhaseSegment1 = (uint8_t) PS1 ;
  //--- Set PS to what is left
    mPropagationSegment = (uint8_t) (propSegmentPlusPhaseSegment1 - PS1) ; // Always 1 <= PropSeg <= 8
  //--- Set SJW to PS2, with a maximum value of 4
    mSJW = (mPhaseSegment2 > 4) ? 4 : (mPhaseSegment2 - 1) ; // Always 2 <= RJW <= 4, and RJW <= mPhaseSegment2
  //--- Triple sampling ?
    mTripleSampling = (inWhishedBitRate <= 125000) && (mPhaseSegment1 >= 2) ;
  //--- Final check of the configuration
    const uint32_t W = bestTQCount * mDesiredBitRate * mBitRatePrescaler * 2 ;
    const uint64_t diff = (mQuartzFrequency > W) ? (mQuartzFrequency - W) : (W - mQuartzFrequency) ;
    const uint64_t ppm = (uint64_t) (1000UL * 1000UL) ; // UL suffix is required for Arduino Uno
    mBitRateClosedToDesiredRate = (diff * ppm) <= (((uint64_t) W) * inTolerancePPM) ;
  }
} ;

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

uint32_t ACANSettings2515::actualBitRate (void) const {
  const uint32_t TQCount = 1 /* Sync Seg */ + mPropagationSegment + mPhaseSegment1 + mPhaseSegment2 ;
  return mQuartzFrequency / mBitRatePrescaler / TQCount / 2 ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

bool ACANSettings2515::exactBitRate (void) const {
  const uint32_t TQCount = 1 /* Sync Seg */ + mPropagationSegment + mPhaseSegment1 + mPhaseSegment2 ;
  return mQuartzFrequency == (mBitRatePrescaler * mDesiredBitRate * TQCount * 2) ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

uint32_t ACANSettings2515::ppmFromDesiredBitRate (void) const {
  const uint32_t TQCount = 1 /* Sync Seg */ + mPropagationSegment + mPhaseSegment1 + mPhaseSegment2 ;
  const uint32_t W = TQCount * mDesiredBitRate * mBitRatePrescaler * 2 ;
  const uint64_t diff = (mQuartzFrequency > W) ? (mQuartzFrequency - W) : (W - mQuartzFrequency) ;
  const uint64_t ppm = (uint64_t) (1000UL * 1000UL) ; // UL suffix is required for Arduino Uno
  return (uint32_t) ((diff * ppm) / W) ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

uint32_t ACANSettings2515::samplePointFromBitStart (void) const {
  const uint32_t TQCount = 1 /* Sync Seg */ + mPropagationSegment + mPhaseSegment1 + mPhaseSegment2 ;
  const uint32_t samplePoint = 1 /* Sync Seg */ + mPropagationSegment + mPhaseSegment1 - mTripleSampling ;
  const uint32_t partPerCent = 100 ;
  return (samplePoint * partPerCent) / TQCount ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

uint32_t ACANSettings2515::CANBitSettingConsistency (void) const {
  uint32_t errorCode = 0 ; // Means no error
  if (mBitRatePrescaler == 0) {
    errorCode |= kBitRatePrescalerIsZero ;
  }else if (mBitRatePrescaler > 64) {
    errorCode |= kBitRatePrescalerIsGreaterThan64 ;
  }
  if (mPropagationSegment == 0) {
    errorCode |= kPropagationSegmentIsZero ;
  }else if (mPropagationSegment > 8) {
    errorCode |= kPropagationSegmentIsGreaterThan8 ;
  }
  if (mPhaseSegment1 == 0) {
    errorCode |= kPhaseSegment1IsZero ;
  }else if ((mPhaseSegment1 == 1) && mTripleSampling) {
    errorCode |= kPhaseSegment1Is1AndTripleSampling ;
  }else if (mPhaseSegment1 > 8) {
    errorCode |= kPhaseSegment1IsGreaterThan8 ;
  }
  if (mPhaseSegment2 < 2) {
    errorCode |= kPhaseSegment2IsLowerThan2 ;
  }else if (mPhaseSegment2 > 8) {
    errorCode |= kPhaseSegment2IsGreaterThan8 ;
  }
  if (mSJW == 0) {
    errorCode |= kSJWIsZero ;
  }else if (mSJW > 4) {
    errorCode |= kSJWIsGreaterThan4 ;
  }
  if (mSJW >= mPhaseSegment2) {
    errorCode |= kSJWIsGreaterThanOrEqualToPhaseSegment2 ;
  }
  if (mPhaseSegment2 > (mPropagationSegment + mPhaseSegment1)) {
    errorCode |= kPhaseSegment2IsGreaterThanPSPlusPS1 ;
  }
  return errorCode ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
