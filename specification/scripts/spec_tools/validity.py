#!/usr/bin/python3 -i
#
# Copyright 2013-2024, The Khronos Group Inc.
#
# SPDX-License-Identifier: Apache-2.0

import re


_A_VS_AN_RE = re.compile(r' a ([a-z]+:)?([aAeEiIoOxX]\w+\b)(?!:)')

_STARTS_WITH_MACRO_RE = re.compile(r'^[a-z]+:.*')

_VUID_ANCHOR_RE = re.compile(r'\[\[VUID-.*\]\]')


def _checkAnchorComponents(anchor):
    """Raise an exception if any component of a VUID anchor name is illegal."""
    if anchor:
        # Any other invalid things in an anchor name should be detected here.
        if any((' ' in anchor_part for anchor_part in anchor)):
            raise RuntimeError("Illegal component of a VUID anchor name!")


def _fix_a_vs_an(s):
    """Fix usage (often generated) of the indefinite article 'a' when 'an' is appropriate.

    Explicitly excludes the markup macros."""
    return _A_VS_AN_RE.sub(r' an \1\2', s)


class ValidityCollection:
    """Combines validity for a single entity."""

    def __init__(self, entity_name=None, conventions=None, strict=True, verbose=False):
        self.entity_name = entity_name
        self.conventions = conventions
        self.lines = []
        self.strict = strict
        self.verbose = verbose

    def possiblyAddExtensionRequirement(self, extension_name, entity_preface):
        """Add an extension-related validity statement if required.

        entity_preface is a string that goes between "must be enabled prior to "
        and the name of the entity, and normally ends in a macro.
        For instance, might be "calling flink:" for a function.
        """
        assert self.conventions
        if extension_name and not self.conventions.is_api_version_name(extension_name):
            msg = 'The {} extension must: be enabled prior to {}{}'.format(
                self.conventions.formatExtension(extension_name), entity_preface, self.entity_name)
            self.addValidityEntry(msg, anchor=('extension', 'notenabled'))

    def addValidityEntry(self, msg, anchor=None):
        """Add a validity entry, optionally with a VUID anchor.

        If any trailing arguments are supplied,
        an anchor is generated by concatenating them with dashes
        at the end of the VUID anchor name.
        """
        if not msg:
            raise RuntimeError("Tried to add a blank validity line!")
        parts = ['*']
        _checkAnchorComponents(anchor)
        if anchor:
            if not self.entity_name:
                raise RuntimeError('Cannot add a validity entry with an anchor to a collection that does not know its entity name.')
            parts.append('[[{}]]'.format(
                '-'.join(['VUID', self.entity_name] + list(anchor))))
        parts.append(msg)
        combined = _fix_a_vs_an(' '.join(parts))
        if combined in self.lines:
            raise RuntimeError("Duplicate validity added!")
        self.lines.append(combined)

    def addText(self, msg):
        """Add already formatted validity text."""
        if self.strict:
            raise RuntimeError('addText called when collection in strict mode')
        if not msg:
            return
        msg = msg.rstrip()
        if not msg:
            return
        self.lines.append(msg)

    def _extend(self, lines):
        lines = list(lines)
        dupes = set(lines).intersection(self.lines)
        if dupes:
            raise RuntimeError("The two sets contain some shared entries! " + str(dupes))
        self.lines.extend(lines)

    def __iadd__(self, other):
        """Perform += with a string, iterable, or ValidityCollection."""
        if other is None:
            pass
        elif isinstance(other, str):
            if self.strict:
                raise RuntimeError(
                    'Collection += a string when collection in strict mode')
            if not other:
                # empty string
                pass
            elif other.startswith('*'):
                # Handle already-formatted
                self.addText(other)
            else:
                # Do the formatting ourselves.
                self.addValidityEntry(other)
        elif isinstance(other, ValidityEntry):
            if other:
                if other.verbose:
                    print(self.entity_name, 'Appending', str(other))
                self.addValidityEntry(str(other), anchor=other.anchor)
        elif isinstance(other, ValidityCollection):
            if self.entity_name == other.entity_name:
                self._extend(other.lines)
            else:
                # Remove foreign anchors - this is presumably an alias
                if other.verbose:
                    print(self.entity_name,
                          'merging with validity for',
                          other.entity_name,
                          'so removing VUID anchor on incoming entries')
                self._extend(_VUID_ANCHOR_RE.sub('', s, 1) for s in other.lines)
        else:
            # Deal with other iterables.
            self._extend(other)

        return self

    def __bool__(self):
        """Is the collection non-empty?"""
        empty = not self.lines
        return not empty

    @property
    def text(self):
        """Access validity statements as a single string or None."""
        if not self.lines:
            return None
        return '\n'.join(self.lines) + '\n'

    def __str__(self):
        """Access validity statements as a single string or empty string."""
        if not self:
            return ''
        return self.text

    def __repr__(self):
        return '<ValidityCollection: {}>'.format(self.lines)


class ValidityEntry:
    """A single validity line in progress."""

    def __init__(self, text=None, anchor=None):
        """Prepare to add a validity entry, optionally with a VUID anchor.

        An anchor is generated by concatenating the elements of the anchor tuple with dashes
        at the end of the VUID anchor name.
        """
        _checkAnchorComponents(anchor)
        if isinstance(anchor, str):
            # anchor needs to be a tuple
            anchor = (anchor,)

        # VUID does not allow special chars except ":"
        if anchor is not None:
            anchor = [(anchor_value.replace('->', '::').replace('.', '::')) for anchor_value in anchor]

        self.anchor = anchor
        self.parts = []
        self.verbose = False
        if text:
            self.append(text)

    def append(self, part):
        """Append a part of a string.

        If this is the first entry part and the part doesn't start
        with a markup macro, the first character will be capitalized."""
        if not self.parts and not _STARTS_WITH_MACRO_RE.match(part):
            self.parts.append(part[:1].upper())
            self.parts.append(part[1:])
        else:
            self.parts.append(part)
        if self.verbose:
            print('ValidityEntry', id(self), 'after append:', str(self))

    def drop_end(self, n):
        """Remove up to n trailing characters from the string."""
        temp = str(self)
        n = min(len(temp), n)
        self.parts = [temp[:-n]]

    def __iadd__(self, other):
        """Perform += with a string,"""
        self.append(other)
        return self

    def __bool__(self):
        """Return true if we have something more than just an anchor."""
        empty = not self.parts
        return not empty

    def __str__(self):
        """Access validity statement as a single string or empty string."""
        if not self:
            raise RuntimeError("No parts added?")
        return ''.join(self.parts).strip()

    def __repr__(self):
        parts = ['<ValidityEntry: ']
        if self:
            parts.append('"')
            parts.append(str(self))
            parts.append('"')
        else:
            parts.append('EMPTY')
        if self.anchor:
            parts.append(', anchor={}'.format('-'.join(self.anchor)))
        parts.append('>')
        return ''.join(parts)
