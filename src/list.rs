use core::{cell::UnsafeCell, marker::PhantomPinned, ptr::NonNull};

type LinkPtr<T> = Option<NonNull<T>>;

struct LinkInner<T> {
    next: LinkPtr<T>,
    _unpin: PhantomPinned,
}

impl<T> Default for LinkInner<T> {
    #[inline]
    fn default() -> Self {
        Self {
            next: None,
            _unpin: PhantomPinned,
        }
    }
}

/// Forward link for an intrusive singly-linked list.
pub struct Link<T> {
    inner: UnsafeCell<LinkInner<T>>,
}

impl<T> Default for Link<T> {
    #[inline]
    fn default() -> Self {
        Self {
            inner: UnsafeCell::default(),
        }
    }
}

impl<T> Link<T> {
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }

    #[inline]
    pub fn next(&self) -> Option<NonNull<T>> {
        unsafe { (*self.inner.get()).next }
    }

    #[inline]
    pub fn set_next(&mut self, ptr: Option<NonNull<T>>) {
        self.inner.get_mut().next = ptr;
    }
}

struct LinksInner<T> {
    next: LinkPtr<T>,
    prev: LinkPtr<T>,
    _unpin: PhantomPinned,
}

impl<T> Default for LinksInner<T> {
    fn default() -> Self {
        Self {
            next: None,
            prev: None,
            _unpin: PhantomPinned,
        }
    }
}

/// Bidirectional links for an intrusive doubly-linked list.
pub struct Links<T> {
    inner: UnsafeCell<LinksInner<T>>,
}

impl<T> Default for Links<T> {
    fn default() -> Self {
        Self {
            inner: UnsafeCell::default(),
        }
    }
}

impl<T> Links<T> {
    #[inline]
    pub fn new() -> Links<T> {
        Self::default()
    }

    #[inline]
    pub fn next(&self) -> Option<NonNull<T>> {
        unsafe { (*self.inner.get()).next }
    }

    #[inline]
    pub fn set_next(&mut self, ptr: Option<NonNull<T>>) {
        self.inner.get_mut().next = ptr;
    }

    #[inline]
    pub fn prev(&self) -> Option<NonNull<T>> {
        unsafe { (*self.inner.get()).prev }
    }

    #[inline]
    pub fn set_prev(&mut self, ptr: Option<NonNull<T>>) {
        self.inner.get_mut().prev = ptr;
    }

    #[inline]
    pub fn clear(&mut self) {
        self.inner.get_mut().prev = None;
        self.inner.get_mut().next = None;
    }
}
