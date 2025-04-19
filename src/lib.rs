// Export modules for testing
pub mod inject;
pub mod version;

// Only include usb module for tests to avoid compilation errors in normal builds
#[cfg(test)]
pub mod usb;

// Include util module
pub mod util;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_version() {
        // Test that version returns a non-empty string
        let version = version::version();
        assert!(!version.is_empty(), "Version should not be empty");
    }

    #[test]
    fn test_speed_enum() {
        // Test that Speed enum values are consistent
        use inject::Speed;
        
        assert_eq!(Speed::Low as u8, 0);
        assert_eq!(Speed::Full as u8, 1);
        assert_eq!(Speed::High as u8, 2);
        assert_eq!(Speed::Auto as u8, 3);
        
        // Test conversion from u8 to Speed
        assert_eq!(Speed::from(0), Speed::Low);
        assert_eq!(Speed::from(1), Speed::Full);
        assert_eq!(Speed::from(2), Speed::High);
        assert_eq!(Speed::from(3), Speed::Auto);
        assert_eq!(Speed::from(4), Speed::Auto); // Default for invalid values
        
        // Test conversion from Speed to u8
        assert_eq!(u8::from(Speed::Low), 0);
        assert_eq!(u8::from(Speed::Full), 1);
        assert_eq!(u8::from(Speed::High), 2);
        assert_eq!(u8::from(Speed::Auto), 3);
    }
}