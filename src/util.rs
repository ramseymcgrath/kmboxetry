//! Utility functions and types

/// Converts a string to title case
pub fn titlecase(s: &str) -> String {
    let mut result = String::with_capacity(s.len());
    let mut capitalize = true;
    
    for c in s.chars() {
        if c.is_whitespace() || c == '-' || c == '_' {
            result.push(c);
            capitalize = true;
        } else if capitalize {
            result.extend(c.to_uppercase());
            capitalize = false;
        } else {
            result.push(c);
        }
    }
    
    result
}

/// Vector-based map implementation
pub mod vec_map {
    /// A simple map implementation using a vector of key-value pairs
    #[derive(Debug, Clone)]
    pub struct VecMap<K, V> {
        entries: Vec<(K, V)>,
    }
    
    impl<K, V> Default for VecMap<K, V> {
        fn default() -> Self {
            VecMap {
                entries: Vec::new(),
            }
        }
    }
    
    impl<K, V> VecMap<K, V> {
        /// Creates a new empty VecMap
        pub fn new() -> Self {
            VecMap {
                entries: Vec::new(),
            }
        }
        
        /// Inserts a key-value pair into the map
        pub fn insert(&mut self, key: K, value: V) -> Option<V>
        where
            K: PartialEq,
        {
            for entry in &mut self.entries {
                if entry.0 == key {
                    return Some(std::mem::replace(&mut entry.1, value));
                }
            }
            self.entries.push((key, value));
            None
        }
        
        /// Gets a reference to the value associated with the key
        pub fn get<Q>(&self, key: &Q) -> Option<&V>
        where
            K: PartialEq<Q>,
        {
            self.entries
                .iter()
                .find(|(k, _)| k == key)
                .map(|(_, v)| v)
        }
        
        /// Gets a mutable reference to the value associated with the key
        pub fn get_mut<Q>(&mut self, key: &Q) -> Option<&mut V>
        where
            K: PartialEq<Q>,
        {
            self.entries
                .iter_mut()
                .find(|(k, _)| k == key)
                .map(|(_, v)| v)
        }
    }
    
    impl<K, V, Q> std::ops::Index<&Q> for VecMap<K, V>
    where
        K: PartialEq<Q>,
    {
        type Output = V;
        
        fn index(&self, key: &Q) -> &Self::Output {
            self.get(key).expect("no entry found for key")
        }
    }
    
    impl<K, V, Q> std::ops::IndexMut<&Q> for VecMap<K, V>
    where
        K: PartialEq<Q>,
    {
        fn index_mut(&mut self, key: &Q) -> &mut Self::Output {
            self.get_mut(key).expect("no entry found for key")
        }
    }
}