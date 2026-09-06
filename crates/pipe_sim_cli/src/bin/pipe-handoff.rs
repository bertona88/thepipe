use pipe_sim::handoff::{controller::Phase, run_stationary_coupon, Fault};
use std::process::ExitCode;

fn main() -> ExitCode {
    let args:Vec<_>=std::env::args().skip(1).collect();
    let fault=match args.as_slice() {
        []=>Fault::None,
        [option,name] if option=="--fault"=>match name.as_str() {
            "observation_lost_before_close"=>Fault::ObservationLostBeforeClose,
            "observation_lost_before_transfer"=>Fault::ObservationLostBeforeTransfer,
            "observation_lost_after_transfer"=>Fault::ObservationLostAfterTransfer,
            "stale_observation"=>Fault::StaleObservation,
            "receiver_contact_missing"=>Fault::ReceiverContactMissing,
            "transfer_rejected"=>Fault::TransferRejected,
            "inconsistent_pose"=>Fault::InconsistentPose,
            _=>{eprintln!("unknown fault: {name}");return ExitCode::FAILURE;}
        },
        _=>{eprintln!("usage: pipe-handoff [--fault NAME]");return ExitCode::FAILURE;}
    };
    match run_stationary_coupon(fault) {
        Ok(report)=>{
            if let Err(error)=serde_json::to_writer(std::io::stdout().lock(),&report) {
                eprintln!("write report: {error}");return ExitCode::FAILURE;
            }
            if report.status==Phase::Complete {ExitCode::SUCCESS} else {ExitCode::from(2)}
        }
        Err(error)=>{eprintln!("handoff unavailable: {error}");ExitCode::FAILURE}
    }
}
