import PhysicalAIParadigmShift from "../paradigms/paradigamsCrad";


const Details = () => {

  return (
    <>
    <section className="flex flex-col items-center text-white text-sm  py-6 relative bg-[url('https://raw.githubusercontent.com/prebuiltui/prebuiltui/main/assets/hero/green-gradient-bg.svg')] bg-top bg-no-repeat">
     

      {/* Hero Content */}
        <div className="flex items-center justify-center mb-6 animate-fadeIn">
          <div className="flex items-center gap-2 text-[12px] text-green-800 bg-[var(--color-badge-background-top)] border border-orange-200 rounded-full px-4 py-1 shadow-sm">
            <svg
              width="16"
              height="16"
              viewBox="0 0 24 24"
              fill="none"
              xmlns="http://www.w3.org/2000/svg"
            >
              <path
                d="M13 2L3 14h9l-1 8 10-12h-9l1-8z"
                stroke="#f97316"
                strokeWidth="2"
                strokeLinecap="round"
                strokeLinejoin="round"
              />
            </svg>
            <span>The Evolution of Robotics</span>
          </div>
        
        <div/>
      </div>
      <h1 className="text-center text-5xl leading-[68px] md:text-4xl md:leading-[70px]  font-semibold max-w-2xl">
        Let's build AI agents together
      </h1>
      <p className="text-center text-bas max-w-lg mt-2">
        Our platform helps you build, test, and deliver faster — so you can
        focus on what matters.
      </p>

            <PhysicalAIParadigmShift/>

      
    </section>
   
    </>
  );
};

export default Details;
