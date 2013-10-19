/******************************************************************************************
* Android Java Interface for Razor AHRS v1.4.2
* 9 Degree of Measurement Attitude and Heading Reference System
* for Sparkfun "9DOF Razor IMU" and "9DOF Sensor Stick"
*
* Released under GNU GPL (General Public License) v3.0
* Copyright (C) 2013 Peter Bartz [http://ptrbrtz.net]
* Copyright (C) 2011-2012 Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin
* Written by Peter Bartz (peter-bartz@gmx.de)
*
* Infos, updates, bug reports, contributions and feedback:
*     https://github.com/ptrbrtz/razor-9dof-ahrs
******************************************************************************************/

package de.tuberlin.qu.razorahrs;

/**
 * A generic high performance object pooling implementation without size limit. Very useful to avoid
 * pauses caused by the Garbage Collector due to object creation and deletion. Will only allocate
 * memory if get() is called on empty pool.
 * 
 * @param <T> Type of pooled objects
 */
public class ObjectPool<T> {
	// Simple linked list
	private class ListElement {
		public T item;
		public ListElement next;
		public ListElement prev;
	}
	private ListElement listHead = new ListElement();
	private ListElement listPosition = listHead;
	
	private ObjectFactory<T> factory;

	/**
	 * Creates an empty pool.
	 * 
	 * @param factory Factory to be used for creating new objects
	 */
	public ObjectPool(ObjectFactory<T> factory) {
		this.factory = factory;
	}
	
	/**
	 * Creates a pool of specified initial size. Choose sufficient
	 * size to avoid later allocations at all.
	 * 
	 * @param factory Factory to be used for creating new objects
	 * @param initialSize Initial number of objects in the pool
	 */
	public ObjectPool(ObjectFactory<T> factory, int initialSize) {
		this.factory = factory;
		for (int i = 0; i < initialSize; i++) {
			put(factory.newObject());
		}
	}
	
	/**
	 * Puts an object into the pool. Pool size is not limited.
	 * 
	 * @param item
	 */
	public synchronized void put(T item) {
		if (listPosition.next == null) {
			listPosition.next = new ListElement();
			listPosition.next.prev = listPosition;
		}
		listPosition = listPosition.next;
		listPosition.item = item;
	}
	
	/**
	 * Removes and returns an object from the pool. If there are no objects
	 * left, a new object will be created using the factory.
	 * 
	 * @return
	 */
	public synchronized T get() {
		T item;
		if (listPosition != listHead) {
			item = listPosition.item;
			listPosition = listPosition.prev;
		} else {
			item = factory.newObject();
		}
		
		return item;
	}
	
	/**
	 * Used to create objects for an object pool.
	 *
	 * @param <T> Type of pooled objects
	 */
	public static interface ObjectFactory<T> {
		public T newObject();
	}
	
	/**
	 * Object wrapper to enable pooling of primitive float values
	 */
	public static class Float {
		public float value;
	}
	/**
	 * Object wrapper to enable pooling of primitive double values
	 */
	public static class Double {
		public double value;
	}
	/**
	 * Object wrapper to enable pooling of primitive byte values
	 */
	public static class Byte {
		public byte value;
	}
	/**
	 * Object wrapper to enable pooling of primitive short values
	 */
	public static class Short {
		public short value;
	}
	/**
	 * Object wrapper to enable pooling of primitive int values
	 */
	public static class Int {
		public int value;
	}
	/**
	 * Object wrapper to enable pooling of primitive long values
	 */
	public static class Long {
		public long value;
	}
	/**
	 * Object wrapper to enable pooling of primitive boolean values
	 */
	public static class Boolean {
		public boolean value;
	}
	/**
	 * Object wrapper to enable pooling of primitive char values
	 */
	public static class Char {
		public char value;
	}
}