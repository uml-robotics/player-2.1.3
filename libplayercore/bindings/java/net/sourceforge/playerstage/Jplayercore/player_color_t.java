/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 1.3.35
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package net.sourceforge.playerstage.Jplayercore;

public class player_color_t {
  private long swigCPtr;
  protected boolean swigCMemOwn;

  protected player_color_t(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(player_color_t obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if(swigCPtr != 0 && swigCMemOwn) {
      swigCMemOwn = false;
      playercore_javaJNI.delete_player_color_t(swigCPtr);
    }
    swigCPtr = 0;
  }

  protected static long[] cArrayUnwrap(player_color_t[] arrayWrapper) {
      long[] cArray = new long[arrayWrapper.length];
      for (int i=0; i<arrayWrapper.length; i++)
        cArray[i] = player_color_t.getCPtr(arrayWrapper[i]);
      return cArray;
  }

  protected static player_color_t[] cArrayWrap(long[] cArray, boolean cMemoryOwn) {
    player_color_t[] arrayWrapper = new player_color_t[cArray.length];
    for (int i=0; i<cArray.length; i++)
      arrayWrapper[i] = new player_color_t(cArray[i], cMemoryOwn);
    return arrayWrapper;
  }

  public void setAlpha(short value) {
    playercore_javaJNI.player_color_t_alpha_set(swigCPtr, this, value);
  }

  public short getAlpha() {
    return playercore_javaJNI.player_color_t_alpha_get(swigCPtr, this);
  }

  public void setRed(short value) {
    playercore_javaJNI.player_color_t_red_set(swigCPtr, this, value);
  }

  public short getRed() {
    return playercore_javaJNI.player_color_t_red_get(swigCPtr, this);
  }

  public void setGreen(short value) {
    playercore_javaJNI.player_color_t_green_set(swigCPtr, this, value);
  }

  public short getGreen() {
    return playercore_javaJNI.player_color_t_green_get(swigCPtr, this);
  }

  public void setBlue(short value) {
    playercore_javaJNI.player_color_t_blue_set(swigCPtr, this, value);
  }

  public short getBlue() {
    return playercore_javaJNI.player_color_t_blue_get(swigCPtr, this);
  }

  public player_color_t() {
    this(playercore_javaJNI.new_player_color_t(), true);
  }

}
