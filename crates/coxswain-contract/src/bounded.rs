/// Fixed-capacity list. Exists so config data needs no allocator and no
/// external dep.
///
/// Only the first `len` elements are live; the tail holds default values and
/// carries no meaning.
#[derive(Copy, Clone)]
pub struct BoundedList<T, const N: usize> {
    items: [T; N],
    len: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CapacityError;

impl core::fmt::Display for CapacityError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("bounded list capacity exceeded")
    }
}

impl<T: Copy + Default, const N: usize> BoundedList<T, N> {
    pub fn new() -> Self {
        Self {
            items: [T::default(); N],
            len: 0,
        }
    }

    pub fn from_slice(slice: &[T]) -> Result<Self, CapacityError> {
        if slice.len() > N {
            return Err(CapacityError);
        }
        let mut list = Self::new();
        list.items[..slice.len()].copy_from_slice(slice);
        list.len = slice.len();
        Ok(list)
    }
}

impl<T, const N: usize> BoundedList<T, N> {
    pub fn push(&mut self, item: T) -> Result<(), CapacityError> {
        if self.len == N {
            return Err(CapacityError);
        }
        self.items[self.len] = item;
        self.len += 1;
        Ok(())
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn as_slice(&self) -> &[T] {
        &self.items[..self.len]
    }
}

impl<T: Copy + Default, const N: usize> Default for BoundedList<T, N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T, const N: usize> core::ops::Deref for BoundedList<T, N> {
    type Target = [T];

    fn deref(&self) -> &[T] {
        self.as_slice()
    }
}

// Manual impl: a derived PartialEq would compare the dead tail, and two lists
// with equal live prefixes must be equal.
impl<T: PartialEq, const N: usize> PartialEq for BoundedList<T, N> {
    fn eq(&self, other: &Self) -> bool {
        self.as_slice() == other.as_slice()
    }
}

// Manual impl for the same reason: only the live slice is shown.
impl<T: core::fmt::Debug, const N: usize> core::fmt::Debug for BoundedList<T, N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_list().entries(self.as_slice()).finish()
    }
}

// Manual serde impls: the wire form is a plain sequence of the live elements,
// so the dead tail never leaks into serialized data.
#[cfg(feature = "serde")]
impl<T: serde::Serialize, const N: usize> serde::Serialize for BoundedList<T, N> {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.collect_seq(self.as_slice())
    }
}

#[cfg(feature = "serde")]
impl<'de, T, const N: usize> serde::Deserialize<'de> for BoundedList<T, N>
where
    T: serde::Deserialize<'de> + Copy + Default,
{
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        struct SeqVisitor<T, const N: usize>(core::marker::PhantomData<T>);

        impl<'de, T, const N: usize> serde::de::Visitor<'de> for SeqVisitor<T, N>
        where
            T: serde::Deserialize<'de> + Copy + Default,
        {
            type Value = BoundedList<T, N>;

            fn expecting(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                write!(f, "a sequence of at most {N} elements")
            }

            fn visit_seq<A: serde::de::SeqAccess<'de>>(
                self,
                mut seq: A,
            ) -> Result<Self::Value, A::Error> {
                let mut list = BoundedList::new();
                while let Some(item) = seq.next_element()? {
                    list.push(item)
                        .map_err(|_| serde::de::Error::invalid_length(N + 1, &self))?;
                }
                Ok(list)
            }
        }

        deserializer.deserialize_seq(SeqVisitor(core::marker::PhantomData))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn push_until_full_then_capacity_error() {
        let mut list: BoundedList<u8, 2> = BoundedList::new();
        assert!(list.is_empty());
        assert_eq!(list.push(1), Ok(()));
        assert_eq!(list.push(2), Ok(()));
        assert_eq!(list.push(3), Err(CapacityError));
        assert_eq!(list.len(), 2);
    }

    #[test]
    fn from_slice_over_capacity_errors() {
        assert_eq!(
            BoundedList::<u8, 2>::from_slice(&[1, 2, 3]),
            Err(CapacityError)
        );
        let list = BoundedList::<u8, 4>::from_slice(&[1, 2]).unwrap();
        assert_eq!(list.as_slice(), &[1, 2]);
    }

    #[test]
    fn equality_ignores_dead_tail() {
        // Constructed directly so the tails differ; the public API always
        // leaves defaults there.
        let a = BoundedList::<u8, 4> {
            items: [1, 2, 0, 0],
            len: 2,
        };
        let b = BoundedList::<u8, 4> {
            items: [1, 2, 9, 9],
            len: 2,
        };
        assert_eq!(a, b);
        let c = BoundedList::<u8, 4> {
            items: [1, 2, 3, 0],
            len: 3,
        };
        assert_ne!(a, c);
    }

    #[test]
    fn deref_gives_slice_access() {
        let list = BoundedList::<u8, 4>::from_slice(&[5, 6]).unwrap();
        assert_eq!(list[0], 5);
        assert_eq!(list.iter().copied().sum::<u8>(), 11);
    }
}
