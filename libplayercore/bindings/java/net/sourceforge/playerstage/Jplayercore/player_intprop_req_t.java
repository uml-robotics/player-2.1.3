/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 1.3.35
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package net.sourceforge.playerstage.Jplayercore;

public class player_intprop_req_t {
  private long swigCPtr;
  protected boolean swigCMemOwn;

  protected player_intprop_req_t(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(player_intprop_req_t obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if(swigCPtr != 0 && swigCMemOwn) {
      swigCMemOwn = false;
      playercore_javaJNI.delete_player_intprop_req_t(swigCPtr);
    }
    swigCPtr = 0;
  }

  protected static long[] cArrayUnwrap(player_intprop_req_t[] arrayWrapper) {
      long[] cArray = new long[arrayWrapper.length];
      for (int i=0; i<arrayWrapper.length; i++)
        cArray[i] = player_intprop_req_t.getCPtr(arrayWrapper[i]);
      return cArray;
  }

  protected static player_intprop_req_t[] cArrayWrap(long[] cArray, boolean cMemoryOwn) {
    player_intprop_req_t[] arrayWrapper = new player_intprop_req_t[cArray.length];
    for (int i=0; i<cArray.length; i++)
      arrayWrapper[i] = new player_intprop_req_t(cArray[i], cMemoryOwn);
    return arrayWrapper;
  }

  public void setKey_count(long value) {
    playercore_javaJNI.player_intprop_req_t_key_count_set(swigCPtr, this, value);
  }

  public long getKey_count() {
    return playercore_javaJNI.player_intprop_req_t_key_count_get(swigCPtr, this);
  }

  public void setKey(String value) {
    playercore_javaJNI.player_intprop_req_t_key_set(swigCPtr, this, value);
  }

  public String getKey() {
    return playercore_javaJNI.player_intprop_req_t_key_get(swigCPtr, this);
  }

  public void setValue(int value) {
    playercore_javaJNI.player_intprop_req_t_value_set(swigCPtr, this, value);
  }

  public int getValue() {
    return playercore_javaJNI.player_intprop_req_t_value_get(swigCPtr, this);
  }

  public player_intprop_req_t() {
    this(playercore_javaJNI.new_player_intprop_req_t(), true);
  }

}
