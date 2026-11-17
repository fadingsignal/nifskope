#ifndef HKPACKFILE_H
#define HKPACKFILE_H

#include "data/niftypes.h"

#include <QByteArray>
#include <QHash>

#include <cstring>

//! @file hkpackfile.h Reader for Havok binary packfiles (as embedded in Fallout 4's bhkPhysicsSystem)

//! A run of array elements in the packfile's data section
struct HkSpan
{
	quint32 start = 0;	//!< offset of the first element in the data section
	quint32 count = 0;	//!< number of elements
};

/*! Read-only view of a Havok binary packfile ("hk_2014.1.0-r1": 64-bit pointers, little-endian).
 *
 *  Offsets passed to the getters are relative to the __data__ section. Every read is bounds-checked: reads past
 *  the end return zero, so callers do not need to validate offsets taken from the file. The packfile's __types__
 *  section is empty in Fallout 4 files, so class layouts are not read from the file (see hknplayout.h).
 *
 *  The reader points into the QByteArray passed to parse(): keep that buffer alive while the reader is used.
 */
class HkPackfile
{
public:
	//! Parses the header, sections and fixups; returns false if this is not a supported packfile
	bool parse( const QByteArray & file );

	template <typename T> T get( quint32 offset ) const
	{
		T v{};
		if ( quint64( offset ) + sizeof( T ) <= quint64( size ) )
			std::memcpy( &v, data + offset, sizeof( T ) );
		return v;
	}
	Vector3 getVector3( quint32 offset ) const
	{
		return Vector3( get<float>( offset ), get<float>( offset + 4 ), get<float>( offset + 8 ) );
	}
	Vector4 getVector4( quint32 offset ) const
	{
		return Vector4( get<float>( offset ), get<float>( offset + 4 ), get<float>( offset + 8 ), get<float>( offset + 12 ) );
	}

	//! Follows the pointer stored at @p offset; returns false for a null pointer
	bool pointer( quint32 offset, quint32 & target ) const
	{
		auto i = pointers.constFind( offset );
		if ( i == pointers.constEnd() )
			return false;
		target = i.value();
		return true;
	}
	//! hkRelArray at @p field: 16-bit count, then a 16-bit offset measured from the field itself
	HkSpan relArray( quint32 field ) const
	{
		return HkSpan{ field + get<quint16>( field + 2 ), get<quint16>( field ) };
	}
	//! hkArray at @p field: pointer, 32-bit count, capacity and flags. Empty if the pointer is null.
	HkSpan array( quint32 field ) const
	{
		HkSpan s;
		if ( pointer( field, s.start ) )
			s.count = quint32( std::max< qint32 >( get<qint32>( field + 8 ), 0 ) );
		return s;
	}
	//! @p s with its count limited to the elements of @p stride bytes that actually fit in the data
	HkSpan clamp( HkSpan s, quint32 stride ) const
	{
		const quint32 fits = ( s.start < size && stride ) ? ( size - s.start ) / stride : 0;
		s.count = std::min( s.count, fits );
		return s;
	}

	//! Class name of the object at @p offset (empty if there is no object there)
	QByteArray className( quint32 offset ) const { return classes.value( offset ); }
	//! Offset of the first object of class @p name, or -1
	quint32 findObject( const char * name ) const;

private:
	const char * data = nullptr;	// start of the __data__ section
	quint32 size = 0;				// size of the object data (up to the local fixups)
	QHash<quint32, quint32> pointers;	// pointer location -> target, from the local and global fixups
	QHash<quint32, QByteArray> classes;	// object offset -> class name, from the virtual fixups
};

#endif
