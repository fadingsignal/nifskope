#include "hkpackfile.h"

//! @file hkpackfile.cpp Havok binary packfile reader

bool HkPackfile::parse( const QByteArray & file )
{
	static const unsigned char magic[8] = { 0x57, 0xE0, 0xE0, 0x57, 0x10, 0xC0, 0xC0, 0x10 };

	const char * f = file.constData();
	const quint32 fileSize = quint32( file.size() );
	auto u32 = [&]( quint32 o ) -> quint32 {
		quint32 v = 0;
		if ( quint64( o ) + 4 <= fileSize )
			std::memcpy( &v, f + o, 4 );
		return v;
	};

	if ( fileSize < 0x40 || std::memcmp( f, magic, 8 ) != 0 )
		return false;
	// layout rules: 8-byte pointers, little-endian
	if ( f[0x10] != 8 || f[0x11] != 1 )
		return false;

	const quint32 version = u32( 0x0C );
	const qint32 numSections = qint32( u32( 0x14 ) );
	qint32 predicatePadding = 0;
	if ( version >= 11 )
		predicatePadding = std::max< qint16 >( qint16( u32( 0x3C ) >> 16 ), 0 );

	// section headers (0x40 bytes each): name (20 bytes), then start, local fixups, global fixups,
	// virtual fixups, exports, imports, end (all relative to the section start)
	quint32 classStart = 0, classEnd = 0;
	quint32 dataStart = 0, localFixups = 0, globalFixups = 0, virtualFixups = 0, exports = 0;
	int dataSection = -1;
	for ( qint32 i = 0; i < numSections && i < 16; i++ ) {
		const quint32 h = 0x40 + quint32( predicatePadding ) + quint32( i ) * 0x40;
		if ( h + 0x40 > fileSize )
			return false;
		const QByteArray name( f + h, qstrnlen( f + h, 19 ) );
		const quint32 start = u32( h + 20 );
		if ( name == "__classnames__" ) {
			classStart = start;
			classEnd = start + u32( h + 24 );
		} else if ( name == "__data__" ) {
			dataSection = int( i );
			dataStart = start;
			localFixups = u32( h + 24 );
			globalFixups = u32( h + 28 );
			virtualFixups = u32( h + 32 );
			exports = u32( h + 36 );
		}
	}
	if ( dataSection < 0 || !classStart || classEnd > fileSize || quint64( dataStart ) + exports > fileSize
		|| localFixups > globalFixups || globalFixups > virtualFixups || virtualFixups > exports ) {
		return false;
	}

	data = f + dataStart;
	size = localFixups;

	// local fixups: (source, target) pairs within the data section; -1 marks padding
	for ( quint32 o = localFixups; o + 8 <= globalFixups; o += 8 ) {
		const qint32 src = qint32( u32( dataStart + o ) );
		if ( src >= 0 )
			pointers.insert( quint32( src ), u32( dataStart + o + 4 ) );
	}
	// global fixups: (source, target section, target); only pointers into the data section are used
	for ( quint32 o = globalFixups; o + 12 <= virtualFixups; o += 12 ) {
		const qint32 src = qint32( u32( dataStart + o ) );
		if ( src >= 0 && qint32( u32( dataStart + o + 4 ) ) == dataSection )
			pointers.insert( quint32( src ), u32( dataStart + o + 8 ) );
	}
	// virtual fixups: (object, class name section, offset of the name in that section)
	for ( quint32 o = virtualFixups; o + 12 <= exports; o += 12 ) {
		const qint32 obj = qint32( u32( dataStart + o ) );
		if ( obj < 0 )
			continue;
		const quint32 nameOffset = classStart + u32( dataStart + o + 8 );
		if ( nameOffset >= classEnd )
			continue;
		classes.insert( quint32( obj ), QByteArray( f + nameOffset, qstrnlen( f + nameOffset, classEnd - nameOffset ) ) );
	}
	return !classes.isEmpty();
}

quint32 HkPackfile::findObject( const char * name ) const
{
	for ( auto i = classes.constBegin(); i != classes.constEnd(); i++ ) {
		if ( i.value() == name )
			return i.key();
	}
	return quint32( -1 );
}
