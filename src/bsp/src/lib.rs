#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(not(feature = "std"), no_main)]
#![feature(type_alias_impl_trait)]

mod boards;
mod fmt;

#[allow(unused_imports)]
pub use boards::*;

#[cfg(test)]
mod tests {
}
