#![no_std]
pub use singleton_procmacro::*;

pub trait Singleton: FnNew {
    fn get() -> &'static mut Self;
}

pub trait FnNew {
    fn new() -> Self;
}

#[cfg(test)]
mod tests {
    use super::*;
    #[derive(Singleton)]
    struct ASD {
        inner: [u8; 512],
    }

    impl ASD {
        fn print(&mut self) {
            println!("hello")
        }
    }

    impl FnNew for ASD {
        fn new() -> Self {
            Self { inner: [0; 512] }
        }
    }

    #[test]
    fn it_works() {
        let asd = ASD::get();
        asd.print();
        println!("addr {:#?}", unsafe {
            core::mem::transmute::<&mut ASD, *const usize>(asd)
        });
        asd.inner[0] = 1;

        let asd = ASD::get();
        println!("addr {:#?}", unsafe {
            core::mem::transmute::<&mut ASD, *const usize>(asd)
        });
        println!("first elem {}", asd.inner[0])
    }
}
