/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 1.3.35
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package net.sourceforge.playerstage.Jplayertcp;

public class playerxdr_function_t {
  private long swigCPtr;
  protected boolean swigCMemOwn;

  protected playerxdr_function_t(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(playerxdr_function_t obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if(swigCPtr != 0 && swigCMemOwn) {
      swigCMemOwn = false;
      playertcp_javaJNI.delete_playerxdr_function_t(swigCPtr);
    }
    swigCPtr = 0;
  }

  public void setInterf(SWIGTYPE_p_uint16_t value) {
    playertcp_javaJNI.playerxdr_function_t_interf_set(swigCPtr, this, SWIGTYPE_p_uint16_t.getCPtr(value));
  }

  public SWIGTYPE_p_uint16_t getInterf() {
    return new SWIGTYPE_p_uint16_t(playertcp_javaJNI.playerxdr_function_t_interf_get(swigCPtr, this), true);
  }

  public void setType(SWIGTYPE_p_uint8_t value) {
    playertcp_javaJNI.playerxdr_function_t_type_set(swigCPtr, this, SWIGTYPE_p_uint8_t.getCPtr(value));
  }

  public SWIGTYPE_p_uint8_t getType() {
    return new SWIGTYPE_p_uint8_t(playertcp_javaJNI.playerxdr_function_t_type_get(swigCPtr, this), true);
  }

  public void setSubtype(SWIGTYPE_p_uint8_t value) {
    playertcp_javaJNI.playerxdr_function_t_subtype_set(swigCPtr, this, SWIGTYPE_p_uint8_t.getCPtr(value));
  }

  public SWIGTYPE_p_uint8_t getSubtype() {
    return new SWIGTYPE_p_uint8_t(playertcp_javaJNI.playerxdr_function_t_subtype_get(swigCPtr, this), true);
  }

  public void setPackfunc(SWIGTYPE_p_f_p_void_size_t_p_void_int__int value) {
    playertcp_javaJNI.playerxdr_function_t_packfunc_set(swigCPtr, this, SWIGTYPE_p_f_p_void_size_t_p_void_int__int.getCPtr(value));
  }

  public SWIGTYPE_p_f_p_void_size_t_p_void_int__int getPackfunc() {
    long cPtr = playertcp_javaJNI.playerxdr_function_t_packfunc_get(swigCPtr, this);
    return (cPtr == 0) ? null : new SWIGTYPE_p_f_p_void_size_t_p_void_int__int(cPtr, false);
  }

  public void setCopyfunc(SWIGTYPE_p_f_p_void_p_q_const__void__unsigned_int value) {
    playertcp_javaJNI.playerxdr_function_t_copyfunc_set(swigCPtr, this, SWIGTYPE_p_f_p_void_p_q_const__void__unsigned_int.getCPtr(value));
  }

  public SWIGTYPE_p_f_p_void_p_q_const__void__unsigned_int getCopyfunc() {
    long cPtr = playertcp_javaJNI.playerxdr_function_t_copyfunc_get(swigCPtr, this);
    return (cPtr == 0) ? null : new SWIGTYPE_p_f_p_void_p_q_const__void__unsigned_int(cPtr, false);
  }

  public void setCleanupfunc(SWIGTYPE_p_f_p_void__void value) {
    playertcp_javaJNI.playerxdr_function_t_cleanupfunc_set(swigCPtr, this, SWIGTYPE_p_f_p_void__void.getCPtr(value));
  }

  public SWIGTYPE_p_f_p_void__void getCleanupfunc() {
    long cPtr = playertcp_javaJNI.playerxdr_function_t_cleanupfunc_get(swigCPtr, this);
    return (cPtr == 0) ? null : new SWIGTYPE_p_f_p_void__void(cPtr, false);
  }

  public void setClonefunc(SWIGTYPE_p_f_p_void__p_void value) {
    playertcp_javaJNI.playerxdr_function_t_clonefunc_set(swigCPtr, this, SWIGTYPE_p_f_p_void__p_void.getCPtr(value));
  }

  public SWIGTYPE_p_f_p_void__p_void getClonefunc() {
    long cPtr = playertcp_javaJNI.playerxdr_function_t_clonefunc_get(swigCPtr, this);
    return (cPtr == 0) ? null : new SWIGTYPE_p_f_p_void__p_void(cPtr, false);
  }

  public void setFreefunc(SWIGTYPE_p_f_p_void__void value) {
    playertcp_javaJNI.playerxdr_function_t_freefunc_set(swigCPtr, this, SWIGTYPE_p_f_p_void__void.getCPtr(value));
  }

  public SWIGTYPE_p_f_p_void__void getFreefunc() {
    long cPtr = playertcp_javaJNI.playerxdr_function_t_freefunc_get(swigCPtr, this);
    return (cPtr == 0) ? null : new SWIGTYPE_p_f_p_void__void(cPtr, false);
  }

  public void setSizeoffunc(SWIGTYPE_p_f_p_void__unsigned_int value) {
    playertcp_javaJNI.playerxdr_function_t_sizeoffunc_set(swigCPtr, this, SWIGTYPE_p_f_p_void__unsigned_int.getCPtr(value));
  }

  public SWIGTYPE_p_f_p_void__unsigned_int getSizeoffunc() {
    long cPtr = playertcp_javaJNI.playerxdr_function_t_sizeoffunc_get(swigCPtr, this);
    return (cPtr == 0) ? null : new SWIGTYPE_p_f_p_void__unsigned_int(cPtr, false);
  }

  public playerxdr_function_t() {
    this(playertcp_javaJNI.new_playerxdr_function_t(), true);
  }

}
