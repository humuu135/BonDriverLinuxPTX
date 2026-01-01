// IBonDriver4.h: IBonDriver4 クラスのインターフェイス
//
/////////////////////////////////////////////////////////////////////////////

#if !defined(_IBONDRIVER4_H_)
#define _IBONDRIVER4_H_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


#include "IBonDriver3.h"


/////////////////////////////////////////////////////////////////////////////
// Bonドライバインタフェース4
/////////////////////////////////////////////////////////////////////////////

class IBonDriver4 : public IBonDriver3
{
public:
// IBonDriver4
	virtual const QWORD GetPreErrorBitCount(void) = 0;
	virtual const QWORD GetPreTotalBitCount(void) = 0;
	virtual const QWORD GetPostErrorBitCount(void) = 0;
	virtual const QWORD GetPostTotalBitCount(void) = 0;
	virtual const QWORD GetErrorBlockCount(void) = 0;
	virtual const QWORD GetTotalBlockCount(void) = 0;

// IBonDriver
	virtual void Release(void) = 0;
};
#endif
