use cortex_m::interrupt::Mutex;
use core::cell::Cell;
use core::future::Future;
use core::pin::Pin;
use core::task::{Waker, RawWaker, RawWakerVTable, Context, Poll};
use core::sync::atomic::{AtomicBool, Ordering};

pub struct SharedWakerVTable {
    pub after_arm: Option<fn()>,
    pub before_wake: Option<fn()>,
}

pub struct SharedWaker {
    inner: Mutex<Cell<Option<Waker>>>,
    vtable: SharedWakerVTable,
}

impl SharedWaker {
    pub const fn new(vtable: SharedWakerVTable) -> Self {
        Self {
            inner: Mutex::new(Cell::new(None)),
            vtable,
        }
    }

    pub fn arm(&self, waker: Waker) {
        cortex_m::interrupt::free(|cs| {
            self.inner.borrow(cs).set(Some(waker));
        });

        if let Some(f) = self.vtable.after_arm {
            f();
        }
    }

    pub fn wake(&self) {
        cortex_m::interrupt::free(|cs| {
            if let Some(waker) = self.inner.borrow(cs).take() {
                if let Some(f) = self.vtable.before_wake {
                    f();
                }

                waker.wake();
            }
        });
    }
}

pub fn block_on<T>(f: impl Future<Output = T>) -> T {
    // NOTE `*const ()` is &AtomicBool
    static VTABLE: RawWakerVTable = {
        unsafe fn clone(p: *const ()) -> RawWaker {
            RawWaker::new(p, &VTABLE)
        }
        unsafe fn wake(p: *const ()) {
            wake_by_ref(p)
        }
        unsafe fn wake_by_ref(p: *const ()) {
            (*(p as *const AtomicBool)).store(true, Ordering::Release)
        }
        unsafe fn drop(_: *const ()) {
            // no-op
        }

        RawWakerVTable::new(clone, wake, wake_by_ref, drop)
    };

    // Move the value to ensure that it is owned
    let mut f = f;
    // Shadow the original binding so that it can't be directly accessed
    // ever again.
    let mut f = unsafe {
        Pin::new_unchecked(&mut f)
    };

    let ready = AtomicBool::new(true);
    let waker = unsafe {
        Waker::from_raw(RawWaker::new(&ready as *const _ as *const _, &VTABLE))
    };

    loop {
        let mut cx = Context::from_waker(&waker);
        if let Poll::Ready(val) = f.as_mut().poll(&mut cx) {
            break val;
        }
    }
}
