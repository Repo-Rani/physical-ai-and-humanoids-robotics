"use client";
import { Button } from "./moving_border";
import { Sparkles, Cpu, Bot, Zap } from "lucide-react";

export function MovingBorderDemo() {
  return (
    <div className="flex items-center justify-center w-full min-h-[200px]">
      <div className="relative group">
        <style>{`
          @keyframes gradient-shift {
            0%, 100% {
              background-position: 0% 50%;
            }
            50% {
              background-position: 100% 50%;
            }
          }

          @keyframes float {
            0%, 100% {
              transform: translateY(0px);
            }
            50% {
              transform: translateY(-3px);
            }
          }

          @keyframes glow-pulse {
            0%, 100% {
              opacity: 0.5;
            }
            50% {
              opacity: 1;
            }
          }

          .enhanced-button-wrapper {
            position: relative;
          }

          .enhanced-button-wrapper::before {
            content: '';
            position: absolute;
            inset: -2px;
            border-radius: 1.75rem;
            padding: 2px;
            background: linear-gradient(
              135deg,
              #15803d,
              #16a34a,
              #22c55e,
              #15803d
            );
            background-size: 300% 300%;
            animation: gradient-shift 3s ease infinite;
            opacity: 0;
            transition: opacity 0.3s ease;
            z-index: -1;
          }

          [data-theme="dark"] .enhanced-button-wrapper::before {
            background: linear-gradient(
              135deg,
              #14532d,
              #16a34a,
              #34d399,
              #14532d
            );
            background-size: 300% 300%;
          }

          .enhanced-button-wrapper:hover::before {
            opacity: 1;
          }

          .icon-float {
            animation: float 2s ease-in-out infinite;
          }

          .glow-text {
            position: relative;
            display: inline-block;
          }

          :root .glow-text {
            color: #15803d;
            font-weight: 700;
            text-shadow: 0 0 10px rgba(21, 128, 61, 0.3);
          }

          [data-theme="dark"] .glow-text {
            color: #34d399;
            font-weight: 700;
            text-shadow: 0 0 15px rgba(52, 211, 153, 0.4);
          }

          .enhanced-button-content {
            background: linear-gradient(
              135deg,
              rgba(21, 128, 61, 0.05),
              rgba(22, 163, 74, 0.05),
              rgba(34, 197, 94, 0.05)
            );
            backdrop-filter: blur(10px);
            border: 2px solid rgba(21, 128, 61, 0.2);
            transition: all 0.3s ease;
          }

          [data-theme="dark"] .enhanced-button-content {
            background: linear-gradient(
              135deg,
              rgba(20, 83, 45, 0.3),
              rgba(22, 163, 74, 0.2),
              rgba(52, 211, 153, 0.1)
            );
            border: 2px solid rgba(52, 211, 153, 0.3);
          }

          .enhanced-button-wrapper:hover .enhanced-button-content {
            transform: translateY(-2px);
            box-shadow: 0 10px 30px rgba(21, 128, 61, 0.2);
          }

          [data-theme="dark"] .enhanced-button-wrapper:hover .enhanced-button-content {
            box-shadow: 0 10px 40px rgba(52, 211, 153, 0.3);
          }

          .icon-wrapper {
            display: inline-flex;
            align-items: center;
            justify-content: center;
            width: 20px;
            height: 20px;
            margin: 0 4px;
          }

          .sparkle-icon {
            animation: glow-pulse 2s ease-in-out infinite;
          }
        `}</style>

        <div className="enhanced-button-wrapper">
          <Button
            borderRadius="1.75rem"
            className="enhanced-button-content bg-white/80 dark:bg-gray-900/80 backdrop-blur-md"
          >
            <div className="flex items-center gap-3 px-4 py-2">
              <Sparkles className="icon-float sparkle-icon w-5 h-5 text-[var(--ifm-color-primary)]" strokeWidth={2} />
              
              <p className="text-[var(--text-color)] text-[15px] font-medium leading-relaxed">
                <span className="glow-text inline-flex items-center gap-1">
                  <Zap className="w-4 h-4 inline" strokeWidth={2.5} />
                  Ready to shift?
                </span>
                {" "}Master Physical AI through{" "}
                <span className="glow-text inline-flex items-center gap-1">
                  <Cpu className="w-4 h-4 inline" strokeWidth={2.5} />
                  simulation-first learning
                </span>
                {" "}and deploy to{" "}
                <span className="glow-text inline-flex items-center gap-1">
                  <Bot className="w-4 h-4 inline" strokeWidth={2.5} />
                  real humanoid robots
                </span>
              </p>

              <Sparkles className="icon-float sparkle-icon w-5 h-5 text-[var(--ifm-color-primary)]" strokeWidth={2} style={{animationDelay: '1s'}} />
            </div>
          </Button>
        </div>
      </div>
    </div>
  );
}