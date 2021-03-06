/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 1.3.35
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package net.sourceforge.playerstage.Jplayercore;

public class DriverTable {
  private long swigCPtr;
  protected boolean swigCMemOwn;

  protected DriverTable(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(DriverTable obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if(swigCPtr != 0 && swigCMemOwn) {
      swigCMemOwn = false;
      playercore_javaJNI.delete_DriverTable(swigCPtr);
    }
    swigCPtr = 0;
  }

  public DriverTable() {
    this(playercore_javaJNI.new_DriverTable(), true);
  }

  public int AddDriver(String name, SWIGTYPE_p_f_p_ConfigFile_int__p_Driver initfunc) {
    return playercore_javaJNI.DriverTable_AddDriver(swigCPtr, this, name, SWIGTYPE_p_f_p_ConfigFile_int__p_Driver.getCPtr(initfunc));
  }

  public DriverEntry GetDriverEntry(String name) {
    long cPtr = playercore_javaJNI.DriverTable_GetDriverEntry(swigCPtr, this, name);
    return (cPtr == 0) ? null : new DriverEntry(cPtr, false);
  }

  public int Size() {
    return playercore_javaJNI.DriverTable_Size(swigCPtr, this);
  }

  public String GetDriverName(int idx) {
    return playercore_javaJNI.DriverTable_GetDriverName(swigCPtr, this, idx);
  }

  public SWIGTYPE_p_p_char SortDrivers() {
    long cPtr = playercore_javaJNI.DriverTable_SortDrivers(swigCPtr, this);
    return (cPtr == 0) ? null : new SWIGTYPE_p_p_char(cPtr, false);
  }

}
