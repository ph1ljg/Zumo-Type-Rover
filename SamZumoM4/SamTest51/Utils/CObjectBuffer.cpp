/* 
* CObjectBuffer.cpp
*
* Created: 18/04/2020 11:27:23
* Author: Phil2
*/

#include "Includes.h"
#include "stdio.h"
// #include "stdlib.h"
// #include "string.h"
// #include "stdio.h"
// #include "CFrskyTelemetry.h"
// #include "CObjectBuffer.h"

// default constructor
template <typename  T>
CObjectBuffer<T>::CObjectBuffer(uint32_t _size)
{
	// set size to 1 more than requested as the byte buffer gives one less byte than requested. We round up to a full
	// multiple of the object size so that we always get aligned elements, which makes the readptr() method possible
    buffer = new ByteBuffer(((_size+1) * sizeof(T)));


} //CObjectBuffer

// default destructor
template <class T>
CObjectBuffer<T>::~CObjectBuffer()
{
	 delete buffer;
} //~CObjectBuffer


// Discards the buffer content, emptying it.
template <class T>
void CObjectBuffer<T>::clear(void)
{
	buffer->clear();
}


// return number of objects available to be read from the front of the queue
template <class T>
uint32_t CObjectBuffer<T>::available(void) const 
{
    return buffer->available() / sizeof(T);
}

// return number of objects that could be written to the back of the queue
template <class T>
uint32_t CObjectBuffer<T>::space(void) const 
{
    return buffer->space() / sizeof(T);
}

// true is available() == 0
template <class T>
bool CObjectBuffer<T>::empty(void) const 
{
    return buffer->empty();
}

// push one object onto the back of the queue
template <class T>
bool CObjectBuffer<T>::push(const T &object) 
{
    if (buffer->space() < sizeof(T)) 
	{
        return false;
    }
    return buffer->write((uint8_t*)&object, sizeof(T)) == sizeof(T);
}

// push N objects onto the back of the queue
template <class T>
bool CObjectBuffer<T>::push(const T *object, uint32_t n) 
{
    if (buffer->space() < n*sizeof(T)) 
	{
        return false;
    }
    return buffer->write((uint8_t*)object, n*sizeof(T)) == n*sizeof(T);
}
    
// throw away an object from the front of the queue
template <class T>
bool CObjectBuffer<T>::pop(void) 
{
    return buffer->advance(sizeof(T));
}

// pop earliest object off the front of the queue
template <class T>
bool CObjectBuffer<T>::pop(T &object) 
{
    if (buffer->available() < sizeof(T)) 
	{
        return false;
    }
    return buffer->read((uint8_t*)&object, sizeof(T)) == sizeof(T);
}


// push_force() is semantically equivalent to:
//   if (!push(t)) { pop(); push(t); }
template <class T>
bool CObjectBuffer<T>::push_force(const T &object) 
{
    if (buffer->space() < sizeof(T)) 
	{
        buffer->advance(sizeof(T));
    }
    return push(object);
}

// push_force() N objects
template <class T>
bool CObjectBuffer<T>::push_force(const T *object, uint32_t n) 
{
    uint32_t _space = buffer->space();
    if (_space < sizeof(T)*n) 
	{
        buffer->advance(sizeof(T)*(n-_space));
    }
    return push(object, n);
}
    
//    peek copies an object out from the front of the queue without advancing the read pointer
template <class T>
bool CObjectBuffer<T>::peek(T &object) 
{
    return buffer->peekbytes((uint8_t*)&object, sizeof(T)) == sizeof(T);
}

// return a pointer to first contiguous array of available  objects. Return nullptr if none available
template <class T>
const T *CObjectBuffer<T>::readptr(uint32_t &n) 
{
    uint32_t avail_bytes = 0;
    const T *ret = (const T *)buffer->readptr(avail_bytes);
    if (!ret || avail_bytes < sizeof(T)) 
	{
        return nullptr;
    }
    n = avail_bytes / sizeof(T);
    return ret;
}

// advance the read pointer (discarding objects)
template <class T>
bool CObjectBuffer<T>::advance(uint32_t n) 
{
    return buffer->advance(n * sizeof(T));
}
    
// update the object at the front of the queue (the one that would  be fetched by pop()) */
template <class T>
bool CObjectBuffer<T>::update(const T &object) 
{
    return buffer->update((uint8_t*)&object, sizeof(T));
}


//====================================================================================================
ByteBuffer::ByteBuffer(uint32_t _size)
{
    buf = (uint8_t*)calloc(1, _size);
    size = buf ? _size : 0;
}

ByteBuffer::~ByteBuffer(void)
{
    free(buf);
}

/*
 * Caller is responsible for locking in set_size()
 */
bool ByteBuffer::set_size(uint32_t _size)
{
    head = tail = 0;
    if (_size != size) 
	{
        free(buf);
        buf = (uint8_t*)calloc(1, _size);
        if (!buf) 
		{
            size = 0;
            return false;
        }

        size = _size;
    }

    return true;
}

uint32_t ByteBuffer::available(void) const
{
    /* use a copy on stack to avoid race conditions of @tail being updated by
     * the writer thread */
    uint32_t _tail = tail;

    if (head > _tail) 
	{
        return size - head + _tail;
    }
    return _tail - head;
}

void ByteBuffer::clear(void)
{
    head = tail = 0;
}

uint32_t ByteBuffer::space(void) const
{
    if (size == 0) 
	{
        return 0;
    }

    /* use a copy on stack to avoid race conditions of @head being updated by
     * the reader thread */
    uint32_t _head = head;
    uint32_t ret = 0;

    if (_head <= tail) 
	{
        ret = size;
    }

    ret += _head - tail - 1;

    return ret;
}

bool ByteBuffer::empty(void) const
{
    return head == tail;
}

uint32_t ByteBuffer::write(const uint8_t *data, uint32_t len)
{
    ByteBuffer::IoVec vec[2];
    const auto n_vec = reserve(vec, len);
    uint32_t ret = 0;

    for (int i = 0; i < n_vec; i++) 
	{
        memcpy(vec[i].data, data + ret, vec[i].len);
        ret += vec[i].len;
    }

    commit(ret);
    return ret;
}

/*
  update bytes at the read pointer. Used to update an object without
  popping it
 */
bool ByteBuffer::update(const uint8_t *data, uint32_t len)
{
    if (len > available()) 
	{
        return false;
    }
    // perform as two memcpy calls
    uint32_t n = size - head;
    if (n > len) 
	{
        n = len;
    }
    memcpy(&buf[head], data, n);
    data += n;
    if (len > n) 
	{
        memcpy(&buf[0], data, len-n);
    }
    return true;
}

bool ByteBuffer::advance(uint32_t n)
{
    if (n > available()) 
	{
        return false;
    }
    head = (head + n) % size;
    return true;
}

uint8_t ByteBuffer::peekiovec(ByteBuffer::IoVec iovec[2], uint32_t len)
{
    uint32_t n = available();

    if (len > n) 
	{
        len = n;
    }
    if (len == 0) 
	{
        return 0;
    }

    auto b = readptr(n);
    if (n == 0) {
        iovec[0].data = buf;
        iovec[0].len = len;
        iovec[1].data = nullptr;
        iovec[1].len = 0;
        return 1;
    }
    if (n > len) 
	{
        n = len;
    }

    iovec[0].data = const_cast<uint8_t *>(b);
    iovec[0].len = n;

    if (len <= n) 
	{
        return 1;
    }

    iovec[1].data = buf;
    iovec[1].len = len - n;

    return 2;
}

/*
  read len bytes without advancing the read pointer
 */
uint32_t ByteBuffer::peekbytes(uint8_t *data, uint32_t len)
{
    ByteBuffer::IoVec vec[2];
    const auto n_vec = peekiovec(vec, len);
    uint32_t ret = 0;

    for (int i = 0; i < n_vec; i++) 
	{
        memcpy(data + ret, vec[i].data, vec[i].len);
        ret += vec[i].len;
    }

    return ret;
}

uint8_t ByteBuffer::reserve(ByteBuffer::IoVec iovec[2], uint32_t len)
{
    uint32_t n = space();

    if (len > n) 
	{
        len = n;
    }

    if (!len) 
	{
        return 0;
    }

    iovec[0].data = &buf[tail];

    n = size - tail;
    if (len <= n) 
	{
        iovec[0].len = len;
        return 1;
    }

    iovec[0].len = n;

    iovec[1].data = buf;
    iovec[1].len = len - n;

    return 2;
}

/*
 * Advance the writer pointer by 'len'
 */
bool ByteBuffer::commit(uint32_t len)
{
    if (len > space()) 
	{
        return false; //Someone broke the agreement
    }

    tail = (tail + len) % size;
    return true;
}

uint32_t ByteBuffer::read(uint8_t *data, uint32_t len)
{
    uint32_t ret = peekbytes(data, len);
    advance(ret);
    return ret;
}

bool ByteBuffer::read_byte(uint8_t *data)
{
    if (!data) 
	{
        return false;
    }

    int16_t ret = peek(0);
    if (ret < 0) 
	{
        return false;
    }

    *data = ret;

    return advance(1);
}

/*
 * Returns the pointer and size to a contiguous read in the buffer
 */
const uint8_t *ByteBuffer::readptr(uint32_t &available_bytes)
{
    uint32_t _tail = tail;
    available_bytes = (head > _tail) ? size - head : _tail - head;

    return available_bytes ? &buf[head] : nullptr;
}

int16_t ByteBuffer::peek(uint32_t ofs) const
{
    if (ofs >= available()) 
	{
        return -1;
    }
    return buf[(head+ofs)%size];
}
template class CObjectBuffer<mavlink_statustext_t>;
