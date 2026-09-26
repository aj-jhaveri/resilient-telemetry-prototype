//! Strictly increasing sequence check, shared by every receiver.

use thiserror::Error;

/// Why a sequence number was refused.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum SequenceError {
    #[error("duplicate sequence {0}")]
    Duplicate(u64),
    #[error("out-of-order sequence {received} (last accepted {last_accepted})")]
    OutOfOrder { received: u64, last_accepted: u64 },
}

/// Accepts the first sequence it sees, then only strictly greater ones.
#[derive(Debug, Default)]
pub struct SequenceGate {
    last_accepted: Option<u64>,
}

impl SequenceGate {
    pub fn last_accepted(&self) -> Option<u64> {
        self.last_accepted
    }

    /// Records `sequence` if it is acceptable. A refused sequence changes
    /// nothing.
    pub fn check_and_record(&mut self, sequence: u64) -> Result<(), SequenceError> {
        if let Some(last_accepted) = self.last_accepted {
            if sequence == last_accepted {
                return Err(SequenceError::Duplicate(sequence));
            }
            if sequence < last_accepted {
                return Err(SequenceError::OutOfOrder {
                    received: sequence,
                    last_accepted,
                });
            }
        }
        self.last_accepted = Some(sequence);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_sequence_of_any_value_is_accepted() {
        let mut gate = SequenceGate::default();
        assert_eq!(gate.check_and_record(42), Ok(()));
        assert_eq!(gate.last_accepted(), Some(42));
    }

    #[test]
    fn strictly_increasing_sequences_with_gaps_are_accepted() {
        let mut gate = SequenceGate::default();
        for sequence in [1, 2, 5, 6] {
            assert_eq!(gate.check_and_record(sequence), Ok(()));
        }
        assert_eq!(gate.last_accepted(), Some(6));
    }

    #[test]
    fn duplicate_and_out_of_order_are_refused_without_recording() {
        let mut gate = SequenceGate::default();
        assert_eq!(gate.check_and_record(5), Ok(()));
        assert_eq!(gate.check_and_record(5), Err(SequenceError::Duplicate(5)));
        assert_eq!(
            gate.check_and_record(4),
            Err(SequenceError::OutOfOrder {
                received: 4,
                last_accepted: 5
            })
        );
        assert_eq!(gate.last_accepted(), Some(5));
    }
}
