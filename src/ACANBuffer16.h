//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// A CAN driver for MCP2517FD CAN Controller in CAN 2.0B mode
// by Pierre Molinaro
// https://github.com/pierremolinaro/acan2517
//
// This file is common with the acan2515 library
// https://github.com/pierremolinaro/acan2515
//
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

#ifndef ACAN_BUFFER_16_CLASS_DEFINED
#define ACAN_BUFFER_16_CLASS_DEFINED

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

#include <CANMessage.h>

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

class ACANBuffer16 {

//······················································································································
// Default constructor
//······················································································································

  public: ACANBuffer16 (void)  :
  mBuffer (NULL),
  mSize (0),
  mReadIndex (0),
//  mWriteIndex (0),
  mCount (0),
  mPeakCount (0) {
  }

//······················································································································
// Destructor
//······················································································································

  public: ~ ACANBuffer16 (void) {
    delete [] mBuffer ;
  }

//······················································································································
// Private properties
//······················································································································

  private: CANMessage * mBuffer ;
  private: uint16_t mSize ;
  private: uint16_t mReadIndex ;
  private: uint16_t mCount ;
  private: uint16_t mPeakCount ; // > mSize if overflow did occur

//······················································································································
// Accessors
//······················································································································

  public: inline uint16_t size (void) const { return mSize ; }
  public: inline uint16_t count (void) const { return mCount ; }
  public: inline uint16_t peakCount (void) const { return mPeakCount ; }

//······················································································································
// initWithSize
//······················································································································

  public: bool initWithSize (const uint16_t inSize) {
    delete [] mBuffer ;
    mBuffer = new CANMessage [inSize] ;
    const bool ok = mBuffer != NULL ;
    mSize = ok ? inSize : 0 ;
    mReadIndex = 0 ;
    mCount = 0 ;
    mPeakCount = 0 ;
    return ok ;
  }

//······················································································································
// append
//······················································································································

  public: bool append (const CANMessage & inMessage) {
    const bool ok = mCount < mSize ;
    if (ok) {
      uint16_t writeIndex = mReadIndex + mCount ;
      if (writeIndex >= mSize) {
        writeIndex -= mSize ;
      }
      mBuffer [writeIndex] = inMessage ;
      mCount ++ ;
      if (mPeakCount < mCount) {
        mPeakCount = mCount ;
      }
    }
    return ok ;
  }

//······················································································································
// Remove
//······················································································································

  public: bool remove (CANMessage & outMessage) {
    const bool ok = mCount > 0 ;
    if (ok) {
      outMessage = mBuffer [mReadIndex] ;
      mCount -= 1 ;
      mReadIndex += 1 ;
      if (mReadIndex == mSize) {
        mReadIndex = 0 ;
      }
    }
    return ok ;
  }

//······················································································································
// Free
//······················································································································

  public: void free (void) {
    delete [] mBuffer ; mBuffer = nullptr ;
    mSize = 0 ;
    mReadIndex = 0 ;
    mCount = 0 ;
    mPeakCount = 0 ;
  }

//······················································································································
// No copy
//······················································································································

  private: ACANBuffer16 (const ACANBuffer16 &) ;
  private: ACANBuffer16 & operator = (const ACANBuffer16 &) ;
} ;

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

#endif
