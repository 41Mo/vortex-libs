#![no_std]

use proc_macro::TokenStream;
use quote::quote;
use syn;


/// Safety:
/// Those, who derive this trate
/// should guarantee, that access to get() is singlethreaded only
#[proc_macro_derive(Singleton)]
pub fn singleton_impl(input: TokenStream) -> TokenStream {
    let ast: syn::DeriveInput = syn::parse(input).unwrap();
    let name = &ast.ident;
    let static_name = quote::format_ident!("_SINGLETON_STATIC_{}", name);
    let gen = quote! {
        impl Singleton for #name {
            fn get() -> &'static mut Self {
                unsafe {
                    static mut #static_name: Option<#name> = None;
                    if #static_name.is_none() {
                        #static_name.replace(#name::new());
                    }
                    #static_name.as_mut().unwrap()
                }
            }
        }
    };
    gen.into()
}

