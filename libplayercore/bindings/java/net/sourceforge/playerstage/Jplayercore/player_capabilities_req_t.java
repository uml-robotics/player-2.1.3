/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 1.3.35
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package net.sourceforge.playerstage.Jplayercore;

public class player_capabilities_req_t {
  private long swigCPtr;
  protected boolean swigCMemOwn;

  protected player_capabilities_req_t(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(player_capabilities_req_t obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if(swigCPtr != 0 && swigCMemOwn) {
      swigCMemOwn = false;
      playercore_javaJNI.delete_player_capabilities_req_t(swigCPtr);
    }
    swigCPtr = 0;
  }

  protected static long[] cArrayUnwrap(player_capabilities_req_t[] arrayWrapper) {
      long[] cArray = new long[arrayWrapper.length];
      for (int i=0; i<arrayWrapper.length; i++)
        cArray[i] = player_capabilities_req_t.getCPtr(arrayWrapper[i]);
      return cArray;
  }

  protected static player_capabilities_req_t[] cArrayWrap(long[] cArray, boolean cMemoryOwn) {
    player_capabilities_req_t[] arrayWrapper = new player_capabilities_req_t[cArray.length];
    for (int i=0; i<cArray.length; i++)
      arrayWrapper[i] = new player_capabilities_req_t(cArray[i], cMemoryOwn);
    return arrayWrapper;
  }

  public void setType(long value) {
    playercore_javaJNI.player_capabilities_req_t_type_set(swigCPtr, this, value);
  }

  public long getType() {
    return playercore_javaJNI.player_capabilities_req_t_type_get(swigCPtr, this);
  }

  public void setSubtype(long value) {
    playercore_javaJNI.player_capabilities_req_t_subtype_set(swigCPtr, this, value);
  }

  public long getSubtype() {
    return playercore_javaJNI.player_capabilities_req_t_subtype_get(swigCPtr, this);
  }

  public player_capabilities_req_t() {
    this(playercore_javaJNI.new_player_capabilities_req_t(), true);
  }

}
