/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 1.3.35
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package net.sourceforge.playerstage.Jplayercore;

public class player_devaddr_t {
  private long swigCPtr;
  protected boolean swigCMemOwn;

  protected player_devaddr_t(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(player_devaddr_t obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if(swigCPtr != 0 && swigCMemOwn) {
      swigCMemOwn = false;
      playercore_javaJNI.delete_player_devaddr_t(swigCPtr);
    }
    swigCPtr = 0;
  }

  protected static long[] cArrayUnwrap(player_devaddr_t[] arrayWrapper) {
      long[] cArray = new long[arrayWrapper.length];
      for (int i=0; i<arrayWrapper.length; i++)
        cArray[i] = player_devaddr_t.getCPtr(arrayWrapper[i]);
      return cArray;
  }

  protected static player_devaddr_t[] cArrayWrap(long[] cArray, boolean cMemoryOwn) {
    player_devaddr_t[] arrayWrapper = new player_devaddr_t[cArray.length];
    for (int i=0; i<cArray.length; i++)
      arrayWrapper[i] = new player_devaddr_t(cArray[i], cMemoryOwn);
    return arrayWrapper;
  }

  public void setHost(long value) {
    playercore_javaJNI.player_devaddr_t_host_set(swigCPtr, this, value);
  }

  public long getHost() {
    return playercore_javaJNI.player_devaddr_t_host_get(swigCPtr, this);
  }

  public void setRobot(long value) {
    playercore_javaJNI.player_devaddr_t_robot_set(swigCPtr, this, value);
  }

  public long getRobot() {
    return playercore_javaJNI.player_devaddr_t_robot_get(swigCPtr, this);
  }

  public void setInterf(int value) {
    playercore_javaJNI.player_devaddr_t_interf_set(swigCPtr, this, value);
  }

  public int getInterf() {
    return playercore_javaJNI.player_devaddr_t_interf_get(swigCPtr, this);
  }

  public void setIndex(int value) {
    playercore_javaJNI.player_devaddr_t_index_set(swigCPtr, this, value);
  }

  public int getIndex() {
    return playercore_javaJNI.player_devaddr_t_index_get(swigCPtr, this);
  }

  public player_devaddr_t() {
    this(playercore_javaJNI.new_player_devaddr_t(), true);
  }

}
