#include "wled.h"
/*
 * Register your v2 usermods here!
 */
#ifdef USERMOD_PROTOFUSION
#include "../usermods/protofusion_v2/usermod_protofusion.h"
#endif

void registerUsermods()
{
#ifdef USERMOD_PROTOFUSION
  UsermodManager::add(new Usermod_Protofusion());
#endif
}