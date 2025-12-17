"use client";
import { Button } from "../ui/moving-border";

export function MovingBorderDemo() {
  return (
    <div>
      <Button
        borderRadius="1.75rem"
        className="bg-[var(--color-bg)] dark:text-white border-orange-500 dark:border-slate-800"
      >
        <p className="text-[var(--color-text)] text-[15px] font-medium">
            <span className="text-orange-600 font-bold">Ready to shift?</span> Master Physical AI through 
            <span className="text-orange-600 font-bold"> simulation-first learning</span> and deploy to 
            <span className="text-orange-600 font-bold"> real humanoid robots</span>
          </p>
      </Button>
    </div>
  );
}